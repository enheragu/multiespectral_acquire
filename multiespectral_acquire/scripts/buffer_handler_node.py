#!/usr/bin/env python3
"""
Generic buffer handler node — buffers any ROS2 message and stores data synchronized
to a main trigger (ImageWithMetadata).

Can be run standalone (parameters from the ROS parameter system) or embedded inside
a compositor process (parameters passed as a plain dict via param_overrides).
"""

import time
import os
import struct
import yaml
import threading
import importlib
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.serialization import deserialize_message

import cv2
import numpy as np
from cv_bridge import CvBridge
from std_msgs.msg import Bool

from multiespectral_acquire.msg import ImageWithMetadata

# Shared pool for disk writes (PNG/bin/yaml). Keeps slow I/O and PNG encoding
# out of the executor thread so high-rate ingestion callbacks are never blocked
# behind a store burst after each trigger.
_STORE_POOL = ThreadPoolExecutor(max_workers=4, thread_name_prefix='store')

# In-flight store jobs are bounded across ALL handlers (shared pool). Each
# pending job pins its message in memory (a visible frame ~5.8 MB, a calib
# cloud ~3.8 MB), so on a disk that can't keep up an unbounded submit queue
# grows until OOM. We warn early and, past a hard cap, DROP the frame instead
# of submitting — shedding load is the correct failure mode for a real-time
# pipeline on a saturated disk (measured: the USB2 HDD does ~21 MB/s vs the
# ~65 MB/s a calibration recording demands, so it would otherwise back up
# without bound). Cap × max payload (~6 MB) bounds the worst-case footprint.
_STORE_BACKLOG_WARN = 32
_STORE_BACKLOG_MAX  = 64


class TimedBuffer:
    """Generic buffer for timestamped raw serialized messages (thread-safe).

    Two modes:
      - Frame-count (time_window_s=0): classic deque(maxlen=max_size), grows/shrinks by frame count.
      - Time-window (time_window_s>0): unbounded deque purged to keep only the last
        time_window_s of data; max_size is a hard safety cap against sensor bursts.
        Call set_time_window() to change the window at runtime (used by master state machine).
    """

    def __init__(self, max_size=100, time_window_s=0.0):
        self._time_window_s = time_window_s
        self._max_items = max_size
        if time_window_s > 0:
            self.buffer = deque()          # time-managed, no maxlen
        else:
            self.buffer = deque(maxlen=max_size)
        self.lock = threading.Lock()
        self._last_add_mono = None

    def set_time_window(self, window_s: float):
        with self.lock:
            self._time_window_s = max(0.0, window_s)

    def trim_before(self, cutoff_ns: int):
        """Drop entries with timestamp < cutoff_ns (PTP sensor time).
        Called after a successful sync so future triggers can't use stale data anyway."""
        with self.lock:
            while self.buffer and self.buffer[0]['timestamp'] < cutoff_ns:
                self.buffer.popleft()

    def add(self, timestamp_ns, data):
        with self.lock:
            self.buffer.append({'timestamp': timestamp_ns, 'data': data})
            self._last_add_mono = time.monotonic()
            if self._time_window_s > 0:
                cutoff_ns = timestamp_ns - int(self._time_window_s * 1e9)
                while len(self.buffer) > 1 and self.buffer[0]['timestamp'] < cutoff_ns:
                    self.buffer.popleft()
            # Hard cap always applies (prevents bursts from growing unbounded)
            while len(self.buffer) > self._max_items:
                self.buffer.popleft()

    def find_closest(self, target_ns, max_diff_sec=0.1):
        with self.lock:
            if not self.buffer:
                return None
            snapshot = list(self.buffer)

        closest = min(snapshot, key=lambda x: abs(x['timestamp'] - target_ns))
        if abs(closest['timestamp'] - target_ns) / 1e9 <= max_diff_sec:
            return closest
        return None

    def snapshot(self):
        with self.lock:
            return list(self.buffer)

    def grow(self, step=20, max_size=200):
        with self.lock:
            if self._time_window_s > 0:
                return self._max_items, False   # no-op in time-window mode
            current = self.buffer.maxlen
            if current >= max_size:
                return current, False
            new_size = min(max_size, current + step)
            self.buffer = deque(self.buffer, maxlen=new_size)
            return new_size, True

    def shrink(self, step=5, min_size=10):
        with self.lock:
            if self._time_window_s > 0:
                return self._max_items, False   # no-op in time-window mode
            current = self.buffer.maxlen
            if current <= min_size:
                return current, False
            new_size = max(min_size, current - step)
            self.buffer = deque(self.buffer, maxlen=new_size)
            return new_size, True

    def is_empty(self):
        with self.lock:
            return len(self.buffer) == 0

    def get_time_since_last_update(self):
        with self.lock:
            if self._last_add_mono is None:
                return None
            return time.monotonic() - self._last_add_mono


class GenericBufferHandler(Node):
    """Buffer handler node — always instantiated by BufferCompositor, never standalone."""

    def __init__(self, node_name: str, namespace: str, param_overrides: dict):
        # No parameter services / rosout: with 15 handlers in one process each
        # node's ~6 parameter services bloat the executor wait set (~90 extra
        # entities). Parameters arrive via param_overrides, logs via stdout.
        super().__init__(node_name, namespace=namespace, use_global_arguments=False,
                         start_parameter_services=False, enable_rosout=False)

        self.handler_type     = param_overrides.get('handler_type', 'generic')
        self.message_type     = self._normalize_msg_type(
                                    param_overrides.get('message_type', ''))
        self.data_topic       = param_overrides.get('data_topic', '')
        self.main_trigger_topic = param_overrides.get('main_trigger_topic', '')
        self.store_data       = param_overrides.get('store_data', True)
        self.use_raw          = bool(param_overrides.get('use_raw', False))
        # Buffer by arrival time instead of header.stamp. Needed for the FLIR
        # A68: its PTP "Automatic" mode never actually syncs, so header.stamp
        # is camera uptime (~hours off). The pre-raw code matched LWIR by
        # arrival time implicitly (ImageWithMetadata has no top-level header).
        self.stamp_from_arrival = bool(param_overrides.get('stamp_from_arrival', False))
        self.max_time_diff    = float(param_overrides.get('max_time_diff', 0.1))
        self.exposure_time_ns = int(param_overrides.get('exposure_time_ns', 0))
        self.storage_path     = param_overrides.get('output_path', '')
        _sync_topic_override  = param_overrides.get('sync_topic', None)
        self.store_all        = (not self.main_trigger_topic) or param_overrides.get('store_all', False)
        _buf_initial          = int(param_overrides.get('buffer_initial_size', 20))
        _buf_max              = int(param_overrides.get('buffer_max_size', 30))
        _trigger_hz           = float(param_overrides.get('expected_trigger_hz', 0.0))
        # Calibration mode: store EVERY frame at native rate, not just the one
        # matched to each trigger. The matched frame keeps the canonical base_name;
        # the intermediate frames that arrived since the previous trigger are
        # flushed as <previous_base>_<n>.
        self.calibration_mode = bool(param_overrides.get('calibration_mode', False))
        # Calibration datasets store pixels RAW as .npy instead of PNG: PNG
        # encoding (~150 ms per 1600×1200 visible frame) saturates the Pi during
        # calib capture, while .npy is a near-free memory dump that is also
        # self-describing (dtype+shape in its header → np.load needs no sidecar).
        # The PNG-encode work moves to offline post-processing. Normal stays PNG.
        self.store_raw = bool(param_overrides.get('store_raw', False))

        # ---- Validation ----
        if not self.data_topic:
            raise RuntimeError('data_topic parameter is required')
        if not self.message_type:
            raise RuntimeError('message_type parameter is required (e.g. sensor_msgs/msg/PointCloud2)')

        self.storage_path_created = False
        self.recording_enabled = False

        # Master state machine (only active when expected_trigger_hz > 0).
        # Controls the TimedBuffer time window to cap memory to what's actually useful:
        #   UNKNOWN/STALLED: 3× interval  (wide safety net at startup or brief drop-out)
        #   LIVE:            1.5× interval (tight — only keep what's needed for the next sync)
        #   DEAD:            1.0× interval (minimal — master is gone, stop accumulating)
        self._trigger_hz = _trigger_hz
        self._trigger_interval_s = (1.0 / _trigger_hz) if _trigger_hz > 0 else 0.0
        self._last_trigger_mono = None
        self._master_state = 'UNKNOWN'

        if _trigger_hz > 0 and self.main_trigger_topic:
            initial_window = self._trigger_interval_s * 3.0   # STALLED-equivalent until first trigger
            self.buffer = TimedBuffer(max_size=_buf_max, time_window_s=initial_window)
            self.get_logger().info(
                f"[{self.data_topic}] time-window mode: trigger={_trigger_hz:.2f} Hz, "
                f"initial window={initial_window:.1f}s, hard cap={_buf_max} frames")
        else:
            self.buffer = TimedBuffer(max_size=_buf_initial)   # legacy frame-count mode

        self.pending_sync = deque(maxlen=10)
        self.pending_sync_lock = threading.Lock()
        self.bridge = CvBridge()
        self._throttle_times = {}

        self.buffer_growth_step = 5
        self.buffer_max_size = _buf_max
        self.buffer_shrink_step = 5
        self.buffer_shrink_threshold = 20
        self._buf_initial = _buf_initial
        self._consecutive_syncs = 0

        # Calibration intermediate-flush state (only used when calibration_mode).
        # Frames between sync N and N+1 are flushed at trigger N+1 named after the
        # PREVIOUS sync base. Relies on the buffer window (>=1.5x trigger interval
        # in LIVE state) spanning one inter-trigger gap so no frame is trimmed
        # before it is flushed.
        self._calib_lock = threading.Lock()
        self._calib_last_flush_ns = 0
        self._calib_prev_base = None

        self.get_logger().info(f"[{self.data_topic}] message_type: {self.message_type}")
        self.get_logger().info(f"[{self.data_topic}] handler_type: {self.handler_type}")
        self.get_logger().info(
            f"[{self.data_topic}] mode: "
            f"{'STORE ALL' if self.store_all else f'main-triggered (max_diff={self.max_time_diff}s)'}")

        self.msg_class = self._get_message_class(self.message_type)
        if self.msg_class is None:
            raise RuntimeError(
                f"[{self.data_topic}] Cannot resolve message class for {self.message_type}")

        # Sync publisher — topic name is auto-generated or overridden
        auto_sync = self.data_topic.rstrip('/') + '_sync'
        self.sync_topic = _sync_topic_override if _sync_topic_override else auto_sync
        self.sync_pub = self.create_publisher(self.msg_class, self.sync_topic, 10)
        self.get_logger().info(f"[{self.data_topic}] Will republish to: {self.sync_topic}")

        self.control_sub = self.create_subscription(
            Bool, '/Multiespectral/recording_enabled',
            self._recording_control_cb, 10)

        _reliable = param_overrides.get('use_reliable_qos', False)
        data_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE if _reliable else ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)
        # raw=True delivers serialized CDR bytes: no Python deserialization at
        # ingest rate (30 Hz LWIR, 5×10 Hz Ouster). Only the matched message
        # (trigger rate, ~1 Hz) is deserialized. Requires header-first message
        # types (Image, PointCloud2, ImageWithMetadata all qualify).
        self.data_sub = self.create_generic_subscription(
            self.data_topic, self.message_type, data_qos,
            self._data_callback, raw=self.use_raw)

        if self.main_trigger_topic and not self.store_all:
            self.main_sub = self.create_subscription(
                ImageWithMetadata, self.main_trigger_topic, self._main_callback, 10)

        # Generic clock-offset correction: if this data source runs a free clock
        # (e.g. Ouster TIME_FROM_INTERNAL_OSC), a publisher exposes the
        # internal->wall offset (std_msgs/Float64, seconds) on clock_offset_topic.
        # We add it to every stamp (buffering + republish) so this source lands on
        # the same wall-clock timescale as the main trigger. Topic-agnostic.
        self._clock_offset_ns = 0
        _clock_offset_topic = param_overrides.get('clock_offset_topic', '')
        if _clock_offset_topic:
            from std_msgs.msg import Float64
            off_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST, depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(
                Float64, _clock_offset_topic, self._clock_offset_cb, off_qos)
            self.get_logger().info(
                f"[{self.data_topic}] clock offset from {_clock_offset_topic}")

        # Topic liveness is updated by the compositor's single shared timer via
        # update_topic_status() — 15 per-handler timers each querying the ROS
        # graph every 2 s were pure overhead.
        self.topic_active = False

    # -------------------------------------------------------------------------
    # Internal helpers
    # -------------------------------------------------------------------------

    def create_generic_subscription(self, topic_name, msg_type_str, qos, callback, raw=False):
        msg_class = self._get_message_class(msg_type_str)
        if msg_class is None:
            raise RuntimeError(f"Cannot resolve message type: {msg_type_str}")

        return self.create_subscription(msg_class, topic_name, callback, qos, raw=raw)

    def _normalize_msg_type(self, msg_type):
        parts = msg_type.split('/')
        if len(parts) == 2:
            return f"{parts[0]}/msg/{parts[1]}"
        return msg_type

    def _get_message_class(self, msg_type_str):
        try:
            from rosidl_runtime_py.utilities import get_message
            return get_message(msg_type_str)
        except Exception as e:
            self.get_logger().warning(
                f"[{self.data_topic}] Failed to get message class for {msg_type_str}: {e}")
            return None

    def _warn_throttle(self, key, interval_sec, msg):
        now = time.monotonic()
        if now - self._throttle_times.get(key, 0) >= interval_sec:
            self._throttle_times[key] = now
            self.get_logger().warning(msg)

    def _log_throttle_count(self, key, interval_sec, build_msg, level='warning'):
        """Count events; emit at most one log line per interval with the count.

        Sync-miss storms (13 handlers × several lines per trigger) used to
        saturate journald/rsyslog, which itself worsened the CPU overload.
        """
        counts = getattr(self, '_log_counts', None)
        if counts is None:
            counts = self._log_counts = {}
        counts[key] = counts.get(key, 0) + 1
        now = time.monotonic()
        if now - self._throttle_times.get(key, 0) >= interval_sec:
            self._throttle_times[key] = now
            n = counts[key]
            counts[key] = 0
            # rclpy caches the severity per logging call site: the same line
            # cannot emit info AND warning ("Logger severity cannot be changed
            # between calls"), hence the explicit branch.
            if level == 'info':
                self.get_logger().info(build_msg(n))
            else:
                self.get_logger().warning(build_msg(n))

    def _stamp_to_ns(self, stamp):
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    # -------------------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------------------

    def update_topic_status(self):
        """Called by the compositor's shared timer.

        Uses count_publishers(): the old check (topic present in
        get_topic_names_and_types) was always true because the handler's OWN
        subscription registers the topic in the graph, so STANDBY never fired.
        """
        is_published = self.count_publishers(self.data_topic) > 0

        if is_published and not self.topic_active:
            self.topic_active = True
            self.get_logger().info(f"[{self.data_topic}] Input topic active. Node ACTIVE")
        elif not is_published and self.topic_active:
            self.topic_active = False
            self.get_logger().warning(
                f"[{self.data_topic}] Input topic NOT published. Node in STANDBY")

        self._update_master_state(trigger_arrived=False)

    # -------------------------------------------------------------------------
    # Master state machine
    # -------------------------------------------------------------------------

    # Window multipliers per state (× trigger interval).
    _MASTER_WINDOWS = {
        'UNKNOWN': 3.0,   # conservative start: don't know yet
        'LIVE':    1.5,   # covers callback delay + max_time_diff margin
        'STALLED': 5.0,   # covers up to 5 unresolved trigger windows (= DEAD threshold at 1 Hz)
        'DEAD':    1.0,   # minimal: master is gone, stop accumulating
    }

    def _update_master_state(self, trigger_arrived: bool):
        """Advance the master state machine and adjust the buffer time window."""
        if self._trigger_hz <= 0 or not self.main_trigger_topic:
            return

        now = time.monotonic()
        if trigger_arrived:
            self._last_trigger_mono = now

        if self._last_trigger_mono is None:
            new_state = 'DEAD'
        else:
            elapsed = now - self._last_trigger_mono
            if elapsed < 2.0 * self._trigger_interval_s:
                new_state = 'LIVE'
            elif elapsed < 5.0 * self._trigger_interval_s:
                new_state = 'STALLED'
            else:
                new_state = 'DEAD'

        if new_state != self._master_state:
            old_state = self._master_state
            self._master_state = new_state
            new_window = self._trigger_interval_s * self._MASTER_WINDOWS[new_state]
            self.buffer.set_time_window(new_window)
            self.get_logger().info(
                f"[{self.data_topic}] master {old_state} → {new_state} "
                f"(window={new_window:.1f}s)")

    def _clock_offset_cb(self, msg):
        self._clock_offset_ns = int(msg.data * 1e9)

    def _recording_control_cb(self, msg):
        if not self.store_data:
            return
        if self.recording_enabled == msg.data:
            return

        self.recording_enabled = msg.data
        if msg.data:
            if not self.storage_path_created:
                try:
                    Path(self.storage_path).mkdir(parents=True, exist_ok=True)
                    self.storage_path_created = True
                    self.get_logger().info(
                        f"[{self.data_topic}] Created storage folder: {self.storage_path}")
                except Exception as e:
                    self.get_logger().error(
                        f"[{self.data_topic}] Failed to create storage folder: {e}")
                    self.recording_enabled = False
                    return
            self.get_logger().info(
                f"[{self.data_topic}] Recording ENABLED -> {self.storage_path}")
        else:
            self.get_logger().info(f"[{self.data_topic}] Recording DISABLED")

    def _data_callback(self, msg):
        """msg is a deserialized message, or raw CDR bytes when use_raw=True."""
        if not self.topic_active:
            return

        if self.stamp_from_arrival:
            timestamp_ns = self.get_clock().now().nanoseconds
        elif self.use_raw:
            timestamp_ns = self._raw_stamp_ns(msg) + self._clock_offset_ns
        else:
            timestamp_ns = self._get_timestamp_ns(msg) + self._clock_offset_ns

        if self.store_all:
            if self.sync_pub is not None:
                try:
                    # publish() accepts raw bytes directly (no re-serialization)
                    self.sync_pub.publish(msg)
                except Exception as e:
                    self._warn_throttle('repub', 10.0,
                        f"[{self.data_topic}] Failed to republish: {e}")

            if self.recording_enabled and self.store_data:
                if not self.storage_path_created:
                    self._warn_throttle('path', 5.0,
                        f"[{self.data_topic}] Recording enabled but storage path not created yet")
                    return
                if self.use_raw:
                    msg = self._deserialize(msg)
                    if msg is None:
                        return
                base_name = self._extract_base_name(msg, timestamp_ns)
                self.store_message(msg, base_name)
        else:
            self.buffer.add(timestamp_ns, msg)
            self._retry_pending_sync_requests()

    def _main_callback(self, msg):
        if not self.topic_active:
            return

        self._update_master_state(trigger_arrived=True)

        camera_timestamp = self._stamp_to_ns(msg.metadata.header.stamp)

        exposure_time = self.exposure_time_ns
        if exposure_time == 0 and hasattr(msg.metadata, 'exposure_time'):
            exposure_time = msg.metadata.exposure_time

        sync_timestamp = (camera_timestamp + (exposure_time // 2)
                          if exposure_time > 0 else camera_timestamp)

        base_name = msg.metadata.img_name
        match = self._find_sync_match(sync_timestamp)

        if match is None:
            signed_diff_ns = self._log_no_match_diagnostics(sync_timestamp)
            if signed_diff_ns is not None and signed_diff_ns > 0:
                self._maybe_grow_buffer_for_late_main(signed_diff_ns)
                self._log_throttle_count('drop_newer', 10.0, lambda n:
                    f"[{self.data_topic}] Dropped {n} syncs in last 10s "
                    f"(buffer already newer), last: '{base_name}'")
            else:
                self._enqueue_pending_sync(sync_timestamp, base_name)
            return

        self._process_matched_message(match['data'], base_name,
                                      sync_timestamp_ns=sync_timestamp,
                                      matched_ts_ns=match['timestamp'])

    # -------------------------------------------------------------------------
    # Sync logic
    # -------------------------------------------------------------------------

    def _find_sync_match(self, sync_timestamp):
        """Return the closest buffered entry {'timestamp','data'} within
        max_time_diff, or None. Callers use entry['data'] (the message/raw bytes)
        and entry['timestamp'] (needed for the calibration intermediate flush)."""
        return self.buffer.find_closest(sync_timestamp, self.max_time_diff)

    def _enqueue_pending_sync(self, sync_timestamp, base_name):
        self._consecutive_syncs = 0
        now_sec = time.monotonic()
        with self.pending_sync_lock:
            if len(self.pending_sync) == self.pending_sync.maxlen:
                dropped = self.pending_sync.popleft()
                age_ms = (now_sec - dropped['created_sec']) * 1000.0
                self._log_throttle_count('pending_full', 10.0, lambda n:
                    f"[{self.data_topic}] Pending queue full ({n} drops in last 10s), "
                    f"last dropped '{dropped['base_name']}' age={age_ms:.1f}ms")
            self.pending_sync.append({
                'sync_timestamp': sync_timestamp,
                'base_name': base_name,
                'created_sec': now_sec})

        self._log_throttle_count('queued', 10.0, lambda n:
            f"[{self.data_topic}] No immediate sync ({n} queued for retry in last 10s), "
            f"last: '{base_name}'")

    def _retry_pending_sync_requests(self):
        now_sec = time.monotonic()
        with self.pending_sync_lock:
            if not self.pending_sync:
                return
            pending = list(self.pending_sync)
            self.pending_sync.clear()

        still_pending = []
        for req in pending:
            match = self._find_sync_match(req['sync_timestamp'])
            if match is None:
                diff_ns = self._get_closest_signed_diff_ns(req['sync_timestamp'])
                if diff_ns is not None and diff_ns > 0:
                    self._maybe_grow_buffer_for_late_main(diff_ns)
                    self._log_throttle_count('pending_ahead', 10.0, lambda n:
                        f"[{self.data_topic}] Dropped {n} pending in last 10s "
                        f"(buffer moved ahead), last: '{req['base_name']}'")
                    continue
                still_pending.append(req)
                continue

            self._process_matched_message(
                match['data'], req['base_name'],
                sync_timestamp_ns=req['sync_timestamp'],
                matched_ts_ns=match['timestamp'])
            age_ms = (now_sec - req['created_sec']) * 1000.0
            self._log_throttle_count('resolved', 10.0, lambda n:
                f"[{self.data_topic}] Resolved {n} pending in last 10s, "
                f"last: '{req['base_name']}' after {age_ms:.1f}ms", level='info')

        if still_pending:
            with self.pending_sync_lock:
                for req in still_pending:
                    if len(self.pending_sync) == self.pending_sync.maxlen:
                        self.pending_sync.popleft()
                    self.pending_sync.append(req)

    def _log_no_match_diagnostics(self, sync_timestamp):
        snapshot = self.buffer.snapshot()
        buffer_size = len(snapshot)
        time_since_update = self.buffer.get_time_since_last_update()

        if buffer_size == 0:
            if time_since_update is None:
                self._log_throttle_count('nomatch_empty', 10.0, lambda n:
                    f"[{self.data_topic}] Buffer empty — no messages received yet "
                    f"({n} misses in last 10s)")
            else:
                self._log_throttle_count('nomatch_stale', 10.0, lambda n:
                    f"[{self.data_topic}] Buffer empty — not updated for "
                    f"{time_since_update:.1f}s (source node may have crashed) "
                    f"({n} misses in last 10s)")
            return None

        actual_closest = min(snapshot, key=lambda x: abs(x['timestamp'] - sync_timestamp))
        diff_ns = actual_closest['timestamp'] - sync_timestamp
        relation = ("newer" if diff_ns > 0 else "older")
        stale = (f", not updated for {time_since_update:.1f}s"
                 if time_since_update and time_since_update > 2.0 else "")
        self._log_throttle_count('nomatch', 10.0, lambda n:
            f"[{self.data_topic}] No sync match ({n} misses in last 10s) — "
            f"buffer_size={buffer_size}, closest is {relation} than main by "
            f"{abs(diff_ns)/1e6:.1f}ms (max={self.max_time_diff*1000:.0f}ms){stale}")
        return diff_ns

    def _get_closest_signed_diff_ns(self, sync_timestamp):
        snapshot = self.buffer.snapshot()
        if not snapshot:
            return None
        closest = min(snapshot, key=lambda x: abs(x['timestamp'] - sync_timestamp))
        return closest['timestamp'] - sync_timestamp

    def _maybe_grow_buffer_for_late_main(self, signed_diff_ns):
        self._consecutive_syncs = 0
        if self._trigger_hz > 0:
            # State machine handles window sizing; reactive frame-count growth is disabled.
            self._warn_throttle('buf_late', 10.0,
                f"[{self.data_topic}] Main late by {signed_diff_ns/1e6:.1f}ms "
                f"(master_state={self._master_state})")
            return
        new_size, grown = self.buffer.grow(
            step=self.buffer_growth_step, max_size=self.buffer_max_size)
        if grown:
            self.get_logger().warning(
                f"[{self.data_topic}] Increased buffer to {new_size} "
                f"(main late by {signed_diff_ns/1e6:.1f}ms)")
        else:
            self._warn_throttle('buf_max', 10.0,
                f"[{self.data_topic}] Main late {signed_diff_ns/1e6:.1f}ms, "
                f"buffer already at max {self.buffer_max_size}")

    def _process_matched_message(self, msg, base_name, sync_timestamp_ns=None,
                                 matched_ts_ns=None):
        # Calibration: before storing the matched (canonical) frame, flush the
        # dense intermediate frames that arrived since the previous trigger as
        # <previous_base>_<n>. Done first so the buffer is intact (it gets trimmed
        # at the end of this method).
        if (self.calibration_mode and self.store_data and self.recording_enabled
                and self.storage_path_created and matched_ts_ns is not None):
            self._flush_calib_intermediates(matched_ts_ns)

        # Raw buffers hold CDR bytes; deserialize only the matched message
        # (trigger rate ~1 Hz) so the frame_id rename and storage work as before.
        if self.use_raw and isinstance(msg, (bytes, bytearray, memoryview)):
            msg = self._deserialize(msg)
            if msg is None:
                return

        # Rewrite the stamp to wall-clock too, so downstream (crop) and storage
        # see the corrected time, not the sensor's free-running clock.
        self._apply_clock_offset(msg)

        if self.sync_pub is not None:
            try:
                if hasattr(msg, 'header'):
                    msg.header.frame_id = base_name
                self.sync_pub.publish(msg)
            except Exception as e:
                self.get_logger().debug(
                    f"[{self.data_topic}] Could not republish sync: {e}")

        if self.recording_enabled and self.store_data:
            if not self.storage_path_created:
                self._warn_throttle('path2', 5.0,
                    f"[{self.data_topic}] Recording enabled but storage path not created yet")
            else:
                self.store_message(msg, base_name)

        # This sync becomes the anchor for the next batch of intermediate frames.
        if self.calibration_mode and matched_ts_ns is not None:
            with self._calib_lock:
                self._calib_prev_base = base_name

        # Eagerly free buffer data that future triggers can't use.
        # Cutoff = oldest still-pending trigger (or this sync) minus max_time_diff.
        # This is safe because triggers advance monotonically in time.
        if sync_timestamp_ns is not None and self._trigger_hz > 0:
            oldest_ns = sync_timestamp_ns
            with self.pending_sync_lock:
                for req in self.pending_sync:
                    oldest_ns = min(oldest_ns, req['sync_timestamp'])
            self.buffer.trim_before(oldest_ns - int(self.max_time_diff * 1e9))

        self._consecutive_syncs += 1
        if self._trigger_hz <= 0 and self._consecutive_syncs >= self.buffer_shrink_threshold:
            # Reactive shrink only in frame-count mode; time-window mode uses state machine.
            self._consecutive_syncs = 0
            new_size, shrunk = self.buffer.shrink(
                step=self.buffer_shrink_step, min_size=self._buf_initial)
            if shrunk:
                self.get_logger().info(
                    f"[{self.data_topic}] Buffer shrunk to {new_size} "
                    f"after {self.buffer_shrink_threshold} consecutive syncs")

    def _apply_clock_offset(self, msg):
        """Shift header.stamp by the source clock offset (Ouster free clock -> wall),
        in place. No-op when no offset is configured or the msg has no header."""
        if self._clock_offset_ns and hasattr(msg, 'header'):
            ns = self._stamp_to_ns(msg.header.stamp) + self._clock_offset_ns
            msg.header.stamp.sec = ns // 1_000_000_000
            msg.header.stamp.nanosec = ns % 1_000_000_000

    def _flush_calib_intermediates(self, matched_ts_ns):
        """Store every buffered frame since the previous trigger as
        <previous_base>_<n> (calibration mode). The matched frame itself is stored
        separately as the canonical base_name by the caller, so it is excluded here
        (strictly older than matched_ts_ns)."""
        snapshot = self.buffer.snapshot()
        with self._calib_lock:
            last = self._calib_last_flush_ns
            prev_base = self._calib_prev_base
            self._calib_last_flush_ns = matched_ts_ns
        if prev_base is None:
            return   # no anchor before the first sync; nothing to attribute yet
        inter = sorted((e for e in snapshot if last < e['timestamp'] < matched_ts_ns),
                       key=lambda e: e['timestamp'])
        for i, entry in enumerate(inter, start=1):
            data = entry['data']
            if self.use_raw and isinstance(data, (bytes, bytearray, memoryview)):
                data = self._deserialize(data)
                if data is None:
                    continue
            self._apply_clock_offset(data)
            self.store_message(data, f"{prev_base}_{i}")

    # -------------------------------------------------------------------------
    # Timestamp / name helpers
    # -------------------------------------------------------------------------

    def _get_timestamp_ns(self, msg):
        if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
            return self._stamp_to_ns(msg.header.stamp)
        return self.get_clock().now().nanoseconds

    @staticmethod
    def _raw_stamp_ns(buf: bytes):
        """Extract header.stamp from serialized CDR bytes without deserializing.

        Layout: 4-byte encapsulation header (byte 1: 1=little-endian), then the
        message body. All raw-enabled types (Image, PointCloud2,
        ImageWithMetadata) start with std_msgs/Header → int32 sec, uint32 nanosec.
        Note: for ImageWithMetadata this reads image.header.stamp = the camera
        timestamp — better than the old typed path, which fell back to arrival
        time because ImageWithMetadata has no top-level 'header' field.
        """
        fmt = '<iI' if buf[1] == 1 else '>iI'
        sec, nanosec = struct.unpack_from(fmt, buf, 4)
        return sec * 1_000_000_000 + nanosec

    def _deserialize(self, buf: bytes):
        try:
            return deserialize_message(bytes(buf), self.msg_class)
        except Exception as e:
            self._warn_throttle('deser', 10.0,
                f"[{self.data_topic}] Failed to deserialize raw message: {e}")
            return None

    def _extract_base_name(self, msg, timestamp_ns):
        if hasattr(msg, 'header') and msg.header.frame_id:
            fid = msg.header.frame_id
            if fid and not fid.startswith(('base_', 'ouster', 'map', 'world')):
                return fid
        if hasattr(msg, 'metadata') and hasattr(msg.metadata, 'img_name'):
            return msg.metadata.img_name
        return str(timestamp_ns)

    # -------------------------------------------------------------------------
    # Storage
    # -------------------------------------------------------------------------

    # Class-wide count of store jobs not yet finished (shared pool).
    _pending_stores = 0
    _pending_stores_lock = threading.Lock()

    def store_message(self, msg, base_name):
        """Hand the (exclusively owned) message to the shared store pool.

        PNG encoding of a 1600×1200 frame takes ~104 ms on the Pi5; doing it
        in the executor thread stalled every ingestion callback after each
        trigger. cv2/file I/O release the GIL, so the pool workers overlap fine.

        Bounded: past _STORE_BACKLOG_MAX in-flight jobs the disk can't keep up,
        so drop (and count) the frame instead of growing the queue toward OOM.
        """
        with GenericBufferHandler._pending_stores_lock:
            pending = GenericBufferHandler._pending_stores
            if pending >= _STORE_BACKLOG_MAX:
                dropped = True
            else:
                GenericBufferHandler._pending_stores += 1
                dropped = False
        if dropped:
            self._log_throttle_count('store_drop', 5.0, lambda n:
                f"[{self.data_topic}] Store backlog ≥{_STORE_BACKLOG_MAX} jobs — "
                f"dropped {n} frames in last 5s (disk slower than data rate)")
            return
        if pending >= _STORE_BACKLOG_WARN:
            self._warn_throttle('store_backlog', 10.0,
                f"[{self.data_topic}] Store backlog ≥{_STORE_BACKLOG_WARN} jobs — disk slower than data rate?")
        _STORE_POOL.submit(self._store_dispatch, msg, base_name)

    def _store_dispatch(self, msg, base_name):
        try:
            if   self.handler_type == 'image':         self._store_image_with_metadata(msg, base_name)
            elif self.handler_type == 'simple_image':  self._store_simple_image(msg, base_name)
            elif self.handler_type == 'pointcloud':    self._store_pointcloud(msg, base_name)
            else:                                      self._store_generic_yaml(msg, base_name)
        finally:
            with GenericBufferHandler._pending_stores_lock:
                GenericBufferHandler._pending_stores -= 1

    def _store_pixels(self, cv_image, base_name):
        """Write the pixel array; return its storage tag ('npy' or 'png').

        Calibration → RAW .npy: no PNG-encode CPU on the (saturated) Pi, and
        self-describing (np.save writes dtype+shape in the header, so np.load
        reconstructs with no sidecar). Normal → PNG (smaller, directly viewable).
        Colour order (bgr8 vs rgb8) is not in the .npy header, so callers still
        record 'encoding' in the yaml.
        """
        if self.store_raw:
            np.save(os.path.join(self.storage_path, base_name), cv_image)  # -> base_name.npy
            return 'npy'
        cv2.imwrite(os.path.join(self.storage_path, f"{base_name}.png"), cv_image)
        return 'png'

    def _store_image_with_metadata(self, msg, base_name):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg.image, desired_encoding='passthrough')
            storage = self._store_pixels(cv_image, base_name)

            metadata = self._msg_to_dict(msg.metadata)
            if storage == 'npy':
                # Pixel layout for offline reconstruction of the raw .npy.
                metadata['image'] = {
                    'storage': 'npy', 'encoding': msg.image.encoding,
                    'width': msg.image.width, 'height': msg.image.height,
                    'step': msg.image.step,
                }
            with open(os.path.join(self.storage_path, f"{base_name}.yaml"), 'w') as f:
                yaml.dump(metadata, f, default_flow_style=False)
        except Exception as e:
            self.get_logger().error(f"[{self.data_topic}] Failed to store image: {e}")

    def _store_simple_image(self, msg, base_name):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            storage = self._store_pixels(cv_image, base_name)

            metadata = {
                'timestamp': self._stamp_to_ns(msg.header.stamp),
                'width': msg.width, 'height': msg.height,
                'encoding': msg.encoding, 'frame_id': msg.header.frame_id,
                'storage': storage,
            }
            with open(os.path.join(self.storage_path, f"{base_name}.yaml"), 'w') as f:
                yaml.dump(metadata, f, default_flow_style=False)
        except Exception as e:
            self.get_logger().error(f"[{self.data_topic}] Failed to store simple image: {e}")

    def _store_pointcloud(self, msg, base_name):
        try:
            # Dump the raw PointCloud2 data buffer verbatim: width*point_step bytes
            # of packed records. This is lossless (keeps x,y,z + intensity, t, ring,
            # reflectivity, ambient, range — everything), ~10x cheaper than
            # np.array(list(pc2.read_points(...))) AND avoids that path's float32
            # cast, which raises on mixed-type 'original' clouds (different dtypes
            # cannot share one float32 array). The sibling .yaml carries the exact
            # field layout (name/offset/datatype/count) + point_step + is_bigendian,
            # which is everything needed to parse the .bin back into a structured
            # array offline.
            np.frombuffer(memoryview(msg.data), dtype=np.uint8).tofile(
                os.path.join(self.storage_path, f"{base_name}.bin"))

            metadata = {
                'storage': 'raw_pointcloud2_data',   # .bin = packed records, layout below
                'timestamp': self._stamp_to_ns(msg.header.stamp),
                'frame_id': msg.header.frame_id,
                'width': msg.width, 'height': msg.height,
                'num_points': int(msg.width) * int(msg.height),
                'point_step': msg.point_step, 'row_step': msg.row_step,
                'is_dense': msg.is_dense, 'is_bigendian': msg.is_bigendian,
                'fields': [{'name': f.name, 'offset': f.offset,
                            'datatype': f.datatype, 'count': f.count}
                           for f in msg.fields]
            }
            with open(os.path.join(self.storage_path, f"{base_name}.yaml"), 'w') as f:
                yaml.dump(metadata, f, default_flow_style=False)
        except Exception as e:
            self.get_logger().error(f"[{self.data_topic}] Failed to store pointcloud: {e}")

    def _store_generic_yaml(self, msg, base_name):
        try:
            data = self._msg_to_dict(msg)
            with open(os.path.join(self.storage_path, f"{base_name}.yaml"), 'w') as f:
                yaml.dump(data, f, default_flow_style=False)
        except Exception as e:
            self.get_logger().error(f"[{self.data_topic}] Failed to store generic YAML: {e}")

    def _msg_to_dict(self, msg):
        if msg is None or isinstance(msg, (int, float, str, bool)):
            return msg
        if isinstance(msg, (list, tuple)):
            return [self._msg_to_dict(i) for i in msg]

        result = {}
        for slot in getattr(msg, '__slots__', []):
            value = getattr(msg, slot)
            if hasattr(value, 'sec') and hasattr(value, 'nanosec'):
                result[slot] = value.sec * 1_000_000_000 + value.nanosec
            else:
                result[slot] = self._msg_to_dict(value)
        return result


