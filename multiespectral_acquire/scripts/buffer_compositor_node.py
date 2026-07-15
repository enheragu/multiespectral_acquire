#!/usr/bin/env python3
"""
Buffer compositor — runs all GenericBufferHandler instances in a single Python
process instead of one process per handler.

Memory saving: ~15 separate Python processes × ~50 MB each  →  1 process × ~120 MB.
All handlers appear as normal nodes in the ROS graph (same names and namespaces).
"""

import os
import sys
import time
import functools
import threading
import yaml
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

try:
    # Event-driven executor: no wait-set rebuild over all entities per message.
    # With 16 nodes in one process the MultiThreadedExecutor rebuilt a ~150-entity
    # wait set ~90×/s, which dominated this process's CPU usage.
    from rclpy.experimental.events_executor import EventsExecutor
except ImportError:
    EventsExecutor = None

from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage

from buffer_handler_node import GenericBufferHandler


def _guard(fn):
    """Wrap an executor callback so a Python exception never escapes into the
    EventsExecutor's C++ dispatch. Un-caught there it becomes a
    pybind11::error_already_set -> std::terminate -> SIGABRT that kills the whole
    compositor process (and every _sync republish with it). We log and continue
    instead; the outer 'resilient spin' try/except CANNOT catch this because the
    callback runs inside C++, not under the Python spin() frame."""
    @functools.wraps(fn)
    def wrapper(self, *args, **kwargs):
        try:
            return fn(self, *args, **kwargs)
        except Exception:
            import traceback
            self.get_logger().error(
                f'Unhandled exception in {fn.__name__} (continuing):\n'
                + traceback.format_exc())
    return wrapper


_IMG  = 'multiespectral_acquire/msg/ImageWithMetadata'
_IMG2 = 'sensor_msgs/msg/Image'
_PC2  = 'sensor_msgs/msg/PointCloud2'
_GPS  = 'sensor_msgs/msg/NavSatFix'
_ODOM = 'nav_msgs/msg/Odometry'
_TEMP = 'temperature_driver/msg/TemperatureHumidity'


class BufferCompositor(Node):
    """Reads compositor-level parameters and spawns all buffer handlers."""

    def __init__(self):
        super().__init__('buffer_compositor')

        self.declare_parameter('output_path', '')
        self.declare_parameter('main_trigger_topic', 'visible_camera/image_with_metadata')
        self.declare_parameter('ouster_exposure_ns', 30000)
        self.declare_parameter('gnss_topic',  '/gnss/fix')
        self.declare_parameter('odom_topic',  '/odometry/combined')
        self.declare_parameter('main_trigger_hz', 0.0)
        self.declare_parameter('calibration_mode', False)
        # When true, the 5 Ouster _sync handlers are dropped here — the C++
        # ouster_sync_node republishes the same ouster/*_sync topics (no GIL, so it
        # recovers the image yield the Python handlers dropped under load).
        self.declare_parameter('disable_ouster_sync', False)

        out         = self.get_parameter('output_path').value
        main        = self.get_parameter('main_trigger_topic').value
        oust_ns     = self.get_parameter('ouster_exposure_ns').get_parameter_value().integer_value
        gnss        = self.get_parameter('gnss_topic').value
        odom        = self.get_parameter('odom_topic').value
        trigger_hz  = self.get_parameter('main_trigger_hz').get_parameter_value().double_value
        self._calib = self.get_parameter('calibration_mode').get_parameter_value().bool_value
        _disable_oust = self.get_parameter('disable_ouster_sync').get_parameter_value().bool_value
        ns          = self.get_namespace()   # inherits /Multiespectral from launch group

        self.get_logger().info(
            f'BufferCompositor starting — ns={ns}, output_path={out}, '
            f'calibration_mode={self._calib}')

        # Common kwargs for Ouster sync buffers (time-window mode when trigger_hz > 0).
        # buffer_max_size is a hard safety cap only; the active window is state-machine-managed.
        # Ouster images ~10 Hz: cap 40 frames (~20 MB/type). Pointcloud cap 30 frames
        # (~15 MB at ~500 KB/scan with azimuth_window active).
        # use_raw: buffer serialized CDR bytes; only the matched message (~1 Hz)
        # is deserialized. Saves the Python deserialization of every incoming
        # message and stores compact bytes instead of Python message objects.
        # clock_offset_topic: the Ouster runs TIME_FROM_INTERNAL_OSC (free clock);
        # ouster_recal_node publishes the internal->wall offset. Handlers add it so
        # the lidar lands on the same wall-clock timescale as the camera trigger.
        # In calibration mode the _sync handlers store the FULL uncropped cloud /
        # images directly (store_data=True) and the crop + _store path is skipped
        # (crop nodes not launched, _store handlers not created below). In normal
        # mode they only republish (store_data=False) and the cropped _store
        # handlers write to disk.
        _oust_store = self._calib
        # The driver publishes RELIABLE in BOTH modes (lidar.launch sets
        # use_system_default_qos=true), so read the cloud/images reliably here too:
        # best-effort loses ~half the large fragmented samples intra-host, halving
        # the rate the buffer can match against and doubling the camera↔lidar sync
        # error (±50 → ±100 ms). Reliable keeps the full 10 Hz in the buffer for a
        # tight match — in normal (1 Hz store) and calib (store-all) alike.
        _oust_reliable = True
        # Calibration STORAGE rate caps. The SENSORS stay full-rate (cloud 10 Hz,
        # LWIR 30 Hz) so the matched-frame sync is unaffected; only the on-disk
        # rate of the dense intermediates is capped, and by TIME (store_max_hz in
        # the handler) so it's robust to input-rate dips — stores min(input, cap),
        # never a fixed fraction. Lidar bundle (cloud + 4 images) → 2 Hz, LWIR →
        # 2 Hz, with the visible trigger at 1 Hz. Sized so calib recording fits the
        # USB2 disk ceiling (~22 MB/s) with margin: ~16 MB/s, complete sets, no drops
        # (the WD50NPZZ/JMS578 caps at ~22 in USB2-fallback; see disk notes).
        _lidar_store_hz = 2.0 if self._calib else 0.0
        _lwir_store_hz  = 2.0 if self._calib else 0.0
        # Lidar FOV-scan-centre time offset, COMPUTED per cloud (not hardcoded).
        # The Ouster stamps each frame at frame start (encoder 0°), but it sweeps
        # the camera FOV partway into the 100 ms / 10 Hz rotation. The pointcloud
        # handler computes (t_min+t_max)/2 of the valid points' per-point 't' field
        # = WHEN it swept the FOV centre, and re-stamps the lidar by it — the exact
        # analogue of the cameras' exposure-centre stamp (trigger + actual
        # exposure/2). All Ouster handlers (cloud + 4 images, same sweep) apply it;
        # the value is computed from the cloud and shared (images carry no per-point
        # time). Removes the systematic cloud↔camera offset, leaving only the
        # inherent ±50 ms 10 Hz quantization.
        # Match the LIDAR to the camera's exposure CENTRE: no exposure_time_ns here,
        # so the handler uses the trigger's metadata.exposure_time (sync target =
        # camera_timestamp + exposure/2 = mid-exposure), consistent with the LWIR.
        # (The old exposure_time_ns=ouster_exposure ≈ 30 µs matched exposure START.)
        oust_img = dict(main_trigger_topic=main, max_time_diff=0.06,
                        store_data=_oust_store, use_raw=True,
                        use_reliable_qos=_oust_reliable,
                        clock_offset_topic='/ouster/clock_offset',
                        apply_lidar_scan_offset=True, store_max_hz=_lidar_store_hz,
                        expected_trigger_hz=trigger_hz, buffer_max_size=40, qos_depth=40)
        oust_pc  = dict(main_trigger_topic=main, max_time_diff=0.06,
                        store_data=_oust_store, use_raw=True,
                        use_reliable_qos=_oust_reliable,
                        clock_offset_topic='/ouster/clock_offset',
                        apply_lidar_scan_offset=True, store_max_hz=_lidar_store_hz,
                        expected_trigger_hz=trigger_hz, buffer_max_size=30, qos_depth=40)

        handler_defs = [
            # ---- Cameras ----
            ('buffer_visible', {                                   # store_all: buffer unused
                'handler_type': 'image', 'message_type': _IMG,
                'data_topic':   'visible_camera/image_with_metadata',
                'output_path':  os.path.join(out, 'visible'),
                'use_raw': True,
            }),
            ('buffer_lwir', {                                      # 30 Hz continuous
                'handler_type': 'image', 'message_type': _IMG,
                'data_topic':   'lwir_camera/image_with_metadata',
                'main_trigger_topic': main, 'max_time_diff': 0.1,
                'output_path':  os.path.join(out, 'lwir'),
                'expected_trigger_hz': trigger_hz, 'buffer_max_size': 90,
                'store_max_hz': _lwir_store_hz,
                'use_reliable_qos': True, 'qos_depth': 40,
                # Pair by the FLIR's software-calibrated hardware timestamp
                # (camera clock → wall-clock, like the Basler; verified ~-13 ms).
                # The A68 can't PTP-lock, so flir_adapter falls back to that
                # calibration and header.stamp is trustworthy — no need for the
                # cruder arrival-time pairing.
                'use_raw': True,
            }),

            # ---- Ouster sync buffers (sync only, no disk write) ----
            # range removed: reconstructs EXACTLY from the pointcloud (range field,
            # verified bit-for-bit). No sync/crop/store → saves ROS transport + USB.
            ('buffer_reflec_sync', {
                'handler_type': 'simple_image', 'message_type': _IMG2,
                'data_topic':   '/ouster/reflec_image',
                'sync_topic':   'ouster/reflec_image_sync',
                'output_path':  os.path.join(out, 'lidar_reflec'), **oust_img,
            }),
            ('buffer_signal_sync', {
                'handler_type': 'simple_image', 'message_type': _IMG2,
                'data_topic':   '/ouster/signal_image',
                'sync_topic':   'ouster/signal_image_sync',
                'output_path':  os.path.join(out, 'lidar_signal'), **oust_img,
            }),
            ('buffer_nearir_sync', {
                'handler_type': 'simple_image', 'message_type': _IMG2,
                'data_topic':   '/ouster/nearir_image',
                'sync_topic':   'ouster/nearir_image_sync',
                'output_path':  os.path.join(out, 'lidar_nearir'), **oust_img,
            }),
            ('buffer_pointcloud_sync', {
                'handler_type': 'pointcloud', 'message_type': _PC2,
                'data_topic':   '/ouster/points',
                'sync_topic':   'ouster/points_sync',
                'output_path':  os.path.join(out, 'lidar_pointcloud'),
                **oust_pc,   # carries use_reliable_qos (calib-gated)
            }),

            # ---- Ouster store buffers (store_all, cropped) ----
            # range_store removed (reconstructed from the pointcloud in post-pro).
            ('buffer_reflec_store', {
                'handler_type': 'simple_image', 'message_type': _IMG2,
                'data_topic':   'ouster/reflec_image_sync_cropped',
                'output_path':  os.path.join(out, 'lidar_reflec'),
            }),
            ('buffer_signal_store', {
                'handler_type': 'simple_image', 'message_type': _IMG2,
                'data_topic':   'ouster/signal_image_sync_cropped',
                'output_path':  os.path.join(out, 'lidar_signal'),
            }),
            ('buffer_nearir_store', {
                'handler_type': 'simple_image', 'message_type': _IMG2,
                'data_topic':   'ouster/nearir_image_sync_cropped',
                'output_path':  os.path.join(out, 'lidar_nearir'),
            }),
            ('buffer_pointcloud_store', {
                'handler_type': 'pointcloud', 'message_type': _PC2,
                'data_topic':   'ouster/points_sync_cropped',
                'output_path':  os.path.join(out, 'lidar_pointcloud'),
            }),

            # ---- Sensor buffers (tiny messages → generous limits) ----
            ('buffer_gnss', {
                'message_type': _GPS, 'data_topic': gnss,
                'main_trigger_topic': main, 'max_time_diff': 1.5,
                'output_path': os.path.join(out, 'gnss'),
                'expected_trigger_hz': trigger_hz, 'buffer_max_size': 60,
            }),
            ('buffer_odom', {
                'message_type': _ODOM, 'data_topic': odom,
                'main_trigger_topic': main, 'max_time_diff': 1.5,
                'output_path': os.path.join(out, 'odom'),
                'expected_trigger_hz': trigger_hz, 'buffer_max_size': 120,
            }),
            ('buffer_dht22', {
                'message_type': _TEMP, 'data_topic': '/dht22/data',
                'main_trigger_topic': main, 'max_time_diff': 1.5,
                'output_path': os.path.join(out, 'dht22'),
                'expected_trigger_hz': trigger_hz, 'buffer_max_size': 60,
            }),
        ]

        # Ouster _sync offloaded to the C++ ouster_sync_node (no GIL → recovers the
        # image yield the GIL-bound Python dropped under the 5-stream firehose). Drop
        # the 5 Python _sync handlers; the C++ node republishes the identical
        # ouster/*_sync topics that the crop → _store chain consumes downstream. The
        # _store handlers stay in Python (they read the cropped topics, ~1 Hz).
        if _disable_oust:
            _oust_sync = {'buffer_reflec_sync', 'buffer_signal_sync',
                          'buffer_nearir_sync', 'buffer_pointcloud_sync'}
            handler_defs = [(n, p) for (n, p) in handler_defs if n not in _oust_sync]

        # Calibration storage. Crop nodes are NOT launched in calib, so the FULL
        # uncropped cloud/images must be stored directly.
        #   - C++ path (_disable_oust, the default): the C++ ouster_sync_node publishes
        #     the uncropped ouster/*_sync but does NOT store. Keep the _store handlers
        #     and repoint them from *_sync_cropped -> *_sync so they store the C++'s
        #     uncropped output (store_all, .npy via store_raw below). PHASE 1: this
        #     stores the trigger-MATCHED frames; the dense calib intermediates
        #     (<base>_<n>, rate-capped) are a pending Phase 2 in the C++ node.
        #   - Legacy Python path (_disable_oust false): the Python _sync handlers store
        #     the uncropped directly, so the cropped _store handlers would double-write
        #     -> drop them (the original behaviour).
        if self._calib:
            if _disable_oust:
                for _n, _p in handler_defs:
                    if _n.endswith('_store'):
                        _dt = _p.get('data_topic', '')
                        if _dt.endswith('_sync_cropped'):
                            _p['data_topic'] = _dt[:-len('_cropped')]  # -> *_sync
                            # The C++ ouster_sync_node publishes *_sync RELIABLE; the
                            # uncropped clouds/images are large and BEST_EFFORT (the
                            # _store default) drops them intra-host. Match RELIABLE +
                            # deep queue so calib stores every matched frame.
                            _p['use_reliable_qos'] = True
                            _p['qos_depth'] = 40
            else:
                handler_defs = [(n, p) for (n, p) in handler_defs
                                if not n.endswith('_store')]
        # Propagate the mode to every handler (enables intermediate-frame storage).
        # store_raw: calib datasets save pixels as .npy (no PNG-encode CPU on the
        # Pi — the encode would saturate it; moved to offline post-processing).
        # Normal datasets stay PNG. Only the image store paths honour the flag.
        for _name, _params in handler_defs:
            _params['calibration_mode'] = self._calib
            _params['store_raw'] = self._calib

        self._handlers = []
        for name, params in handler_defs:
            try:
                h = GenericBufferHandler(
                    node_name=name, namespace=ns, param_overrides=params)
                self._handlers.append(h)
            except RuntimeError as e:
                self.get_logger().error(f'Failed to create {name}: {e}')

        self.get_logger().info(
            f'BufferCompositor ready — {len(self._handlers)} handlers active')

        # ---- TF static collection + auto-save on recording start ----
        self._output_path = out
        self._static_transforms = []
        self._static_tf_lock = threading.Lock()

        tf_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=100)
        self._tf_static_sub = self.create_subscription(
            TFMessage, '/tf_static', self._tf_static_cb, tf_qos)

        # Ouster sensor metadata (beam intrinsics, column_window, data format).
        # Saved once per session — required to parse the raw cloud .bin and to
        # reconstruct XYZ from the range images. Latched (TRANSIENT_LOCAL).
        self._ouster_metadata = None
        meta_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        self._ouster_meta_sub = self.create_subscription(
            String, '/ouster/metadata', self._ouster_meta_cb, meta_qos)

        self._recording_sub = self.create_subscription(
            Bool, 'recording_enabled', self._recording_cb, 10)

        # ONE shared topic-status timer for all handlers, instead of 15 timers
        # each dumping the full ROS graph every 2 s.
        self._topic_check_timer = self.create_timer(2.0, self._check_topics_cb)

        # ---- Dataset-disk health gate ----
        # The dataset disk is an autofs automount (fstab x-systemd.automount): when
        # the drive is absent the mount POINT still exists but any write fails with
        # ENODEV. os.path.ismount() can't tell (it's True for the autofs stub even
        # with nothing mounted underneath), so we probe with a real write. The
        # result is latched on /Multiespectral/disk_writable — every handler gates
        # its disk writes on it, so a missing disk degrades to "republish only"
        # instead of spamming ENODEV or (via an un-caught error) crashing. We also
        # publish a human-facing /Multiespectral/recording_status for the GUIs.
        self._dataset_root = self._find_mount_root(out) if out else ''
        self._disk_ok = None                 # None = not probed yet
        self._recording_requested = False
        _latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self._disk_pub   = self.create_publisher(Bool,   'disk_writable',    _latched)
        self._status_pub = self.create_publisher(String, 'recording_status', _latched)
        # The write-probe can block up to the autofs device-timeout (~5 s) when the
        # drive is absent, so it runs in a DEDICATED thread — never on the executor
        # thread, which must stay free to keep republishing the _sync topics.
        # rclpy publishers are thread-safe. One synchronous probe first so the
        # latched state is correct from the start.
        self._update_disk_state()
        self._disk_monitor_run = True
        self._disk_thread = threading.Thread(target=self._disk_monitor_loop, daemon=True)
        self._disk_thread.start()

    def _disk_monitor_loop(self):
        while self._disk_monitor_run and rclpy.ok():
            time.sleep(5.0)
            try:
                self._update_disk_state()
            except Exception:
                import traceback
                self.get_logger().error(
                    'disk monitor error (continuing):\n' + traceback.format_exc())

    @staticmethod
    def _find_mount_root(path):
        """First ancestor of `path` that is a mount point (the autofs stub for the
        dataset disk counts — it IS the mount location, even when the real fs
        underneath isn't mounted). Falls back to '/' if none is found."""
        p = os.path.abspath(path)
        while p != '/' and not os.path.ismount(p):
            p = os.path.dirname(p)
        return p

    def _dataset_writable(self):
        """True iff the dataset disk is really mounted AND accepting writes.
        A real write is the only reliable probe: os.path.ismount() returns True for
        the autofs stub, and ENODEV only surfaces on actual I/O. Never raises."""
        root = self._dataset_root
        if not root or root == '/':
            return False if root == '/' else True   # '' → not configured, don't gate
        probe = os.path.join(root, '.hitos_write_probe')
        try:
            with open(probe, 'wb') as f:
                f.write(b'ok')
            os.remove(probe)
            return True
        except Exception:
            return False

    def _update_disk_state(self):
        ok = self._dataset_writable()
        if ok != self._disk_ok:
            self._disk_ok = ok
            self._disk_pub.publish(Bool(data=ok))
            if ok:
                self.get_logger().info(
                    f'Dataset disk WRITABLE ({self._dataset_root}) — recording to disk enabled')
                # Re-save the session metadata skipped while the disk was down.
                if self._recording_requested:
                    self._save_tf_static()
                    self._save_ouster_metadata()
            else:
                self.get_logger().error(
                    f'Dataset disk NOT mounted/writable ({self._dataset_root}) — '
                    'NOT recording to disk (topics still republished)')
        self._publish_status()

    def _publish_status(self):
        if not self._recording_requested:
            state = 'IDLE'
        elif self._disk_ok:
            state = 'RECORDING'
        else:
            state = 'NO_DISK'          # recording requested but disk unwritable
        self._status_pub.publish(String(data=state))

    @_guard
    def _check_topics_cb(self):
        for h in self._handlers:
            try:
                h.update_topic_status()
            except Exception as e:
                self.get_logger().warning(
                    f'update_topic_status failed for {h.get_name()}: {e}')

    @_guard
    def _tf_static_cb(self, msg):
        with self._static_tf_lock:
            for t in msg.transforms:
                # Replace existing entry for same parent+child pair
                self._static_transforms = [
                    x for x in self._static_transforms
                    if not (x['header']['frame_id'] == t.header.frame_id
                            and x['child_frame_id'] == t.child_frame_id)
                ]
                self._static_transforms.append({
                    'header': {
                        'frame_id': t.header.frame_id,
                        'stamp': t.header.stamp.sec + t.header.stamp.nanosec * 1e-9,
                    },
                    'child_frame_id': t.child_frame_id,
                    'transform': {
                        'translation': {
                            'x': float(t.transform.translation.x),
                            'y': float(t.transform.translation.y),
                            'z': float(t.transform.translation.z),
                        },
                        'rotation': {
                            'x': float(t.transform.rotation.x),
                            'y': float(t.transform.rotation.y),
                            'z': float(t.transform.rotation.z),
                            'w': float(t.transform.rotation.w),
                        },
                    },
                })

    @_guard
    def _ouster_meta_cb(self, msg):
        self._ouster_metadata = msg.data

    @_guard
    def _recording_cb(self, msg):
        self._recording_requested = bool(msg.data)
        if msg.data:
            if self._disk_ok:
                self._save_tf_static()
                self._save_ouster_metadata()
            else:
                self.get_logger().error(
                    'Recording requested but dataset disk NOT writable — nothing '
                    'will be saved until the disk is mounted (see recording_status=NO_DISK)')
        self._publish_status()

    def _save_ouster_metadata(self):
        if not self._ouster_metadata:
            self.get_logger().warning(
                'Ouster metadata not received yet — skipping save')
            return
        try:
            Path(self._output_path).mkdir(parents=True, exist_ok=True)
            out_file = os.path.join(self._output_path, 'ouster_metadata.json')
            with open(out_file, 'w') as f:
                f.write(self._ouster_metadata)
            self.get_logger().info(f'Saved Ouster metadata -> {out_file}')
        except Exception as e:
            self.get_logger().error(f'Failed to save Ouster metadata: {e}')

    def _save_tf_static(self):
        with self._static_tf_lock:
            transforms = list(self._static_transforms)

        if not transforms:
            self.get_logger().warning('TF static: no transforms received yet — skipping save')
            return

        try:
            Path(self._output_path).mkdir(parents=True, exist_ok=True)
            out_file = os.path.join(self._output_path, 'tf_static.yaml')
            with open(out_file, 'w') as f:
                yaml.dump({'transforms': transforms}, f, default_flow_style=False)
            self.get_logger().info(
                f'Saved {len(transforms)} static transforms -> {out_file}')
        except Exception as e:
            self.get_logger().error(f'Failed to save tf_static: {e}')


def main(args=None):
    # Ensure buffer_handler_node.py is importable from the same scripts/ directory
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

    rclpy.init(args=args)

    compositor = BufferCompositor()

    if EventsExecutor is not None:
        executor = EventsExecutor()
        compositor.get_logger().info('Using EventsExecutor')
    else:
        executor = MultiThreadedExecutor()
        compositor.get_logger().warning(
            'EventsExecutor not available — falling back to MultiThreadedExecutor')
    executor.add_node(compositor)
    for handler in compositor._handlers:
        executor.add_node(handler)

    try:
        # Resilient spin: an exception inside ONE callback must not take down
        # the whole 16-node process. Log it and keep spinning.
        while rclpy.ok():
            try:
                executor.spin()
                break
            except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
                break
            except Exception:
                import traceback
                compositor.get_logger().error(
                    'Unhandled exception in callback (continuing):\n'
                    + traceback.format_exc())
    except KeyboardInterrupt:
        pass
    finally:
        try:
            for handler in compositor._handlers:
                handler.destroy_node()
            compositor.destroy_node()
        except KeyboardInterrupt:
            pass  # rclpy races SIGINT during destroy_when_not_in_use — harmless
        except Exception as e:
            import traceback
            print(f'[buffer_compositor] Unexpected error during shutdown: {e}')
            traceback.print_exc()
        # try_shutdown: SIGINT/SIGTERM handlers may have shut the context down
        # already; a second rclpy.shutdown() raises RCLError.
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
