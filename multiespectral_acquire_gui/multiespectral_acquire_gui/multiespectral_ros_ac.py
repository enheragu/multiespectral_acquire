#!/usr/bin/env python3
# encoding: utf-8

import time
import cv2
import base64
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge, CvBridgeError

from multiespectral_acquire.msg import ImageWithMetadata
from multiespectral_acquire_gui.FreqCounter import FreqCounter


class RosMultiespectralAcquire(Node):
    """ROS2 node that bridges camera topics to the Flask/SocketIO GUI."""

    def __init__(self, socketio):
        super().__init__('multiespectral_flask_gui')
        self.socketio = socketio
        self._running = True
        self._bridge = CvBridge()
        self._throttle_times = {}

        # ---- Parameters ----
        self.declare_parameter('gui_title', 'Camera GUI')
        self.declare_parameter('flask_host', '0.0.0.0')
        self.declare_parameter('flask_port', 5000)
        self.declare_parameter('camera1_topic',
            '/Multiespectral/lwir_camera/image_with_metadata_sync')
        self.declare_parameter('camera2_topic',
            '/Multiespectral/visible_camera/image_with_metadata_sync')
        self.declare_parameter('lidar_topic',
            '/Multiespectral/ouster/reflec_image_sync_cropped_sync')
        self.declare_parameter('camera1_name', 'Camera 1')
        self.declare_parameter('camera2_name', 'Camera 2')
        self.declare_parameter('lidar_name', 'LIDAR Image')
        self.declare_parameter('lidar_topic_names', [
            '/Multiespectral/ouster/reflec_image_sync_cropped_sync',
            '/Multiespectral/ouster/signal_image_sync_cropped_sync',
            '/Multiespectral/ouster/nearir_image_sync_cropped_sync',
        ])
        self.declare_parameter('lidar_topic_labels', [
            'Reflectivity Image', 'Signal Image', 'Near-IR Image'
        ])

        p = lambda name: self.get_parameter(name).get_parameter_value()
        self.gui_title           = p('gui_title').string_value
        self.camera1_display_name = p('camera1_name').string_value
        self.camera2_display_name = p('camera2_name').string_value
        self.lidar_display_name  = p('lidar_name').string_value
        camera1_topic = p('camera1_topic').string_value
        camera2_topic = p('camera2_topic').string_value
        lidar_topic   = p('lidar_topic').string_value
        topic_names   = list(p('lidar_topic_names').string_array_value)
        topic_labels  = list(p('lidar_topic_labels').string_array_value)
        self.lidar_topic_options = [
            {'topic': t, 'label': l} for t, l in zip(topic_names, topic_labels)
        ]

        self.get_logger().info(f'GUI Title: "{self.gui_title}"')
        self.get_logger().info(f'Camera 1: "{self.camera1_display_name}" -> {camera1_topic}')
        self.get_logger().info(f'Camera 2: "{self.camera2_display_name}" -> {camera2_topic}')
        self.get_logger().info(f'LIDAR: "{self.lidar_display_name}" -> {lidar_topic}')

        # ---- State ----
        self._image_size = {
            'camera1': (320, 240), 'camera2': (320, 240), 'lidar': (320, 240)
        }
        # Latest JPEG bytes + seq per stream, served via MJPEG (see
        # multiespectral_control.py /stream/<name>). Images are NOT pushed over
        # Socket.IO anymore: a slow remote client (Husarnet relay) made the
        # server-side emit queue grow unbounded (~6 MB/min memory leak). MJPEG
        # keeps only the latest frame → lossy for slow clients, zero accumulation.
        self._frame_lock = threading.Lock()
        self._frames     = {'camera1': None, 'camera2': None, 'lidar': None}
        self._frame_seq  = {'camera1': 0, 'camera2': 0, 'lidar': 0}
        self._recording_enabled = False
        self._lidar_available   = False
        self.current_lidar_topic = lidar_topic

        self._freq_cam1  = FreqCounter()
        self._freq_cam2  = FreqCounter()
        self._freq_lidar = FreqCounter()
        self._freq_cam1.start()
        self._freq_cam2.start()

        self._saved_cam1  = 0
        self._saved_cam2  = 0
        self._saved_lidar = 0

        # ---- Recording publisher / subscriber ----
        self._recording_topic = '/Multiespectral/recording_enabled'
        self._recording_pub = self.create_publisher(Bool, self._recording_topic, 1)
        self._recording_sub = self.create_subscription(
            Bool, self._recording_topic, self._recording_status_cb, 1)

        # ---- Storage/disk status from the compositor ----
        # The compositor is the only disk writer and knows the REAL state: IDLE /
        # RECORDING / NO_DISK (recording requested but the dataset disk isn't
        # mounted). Without this the GUI would show 'RECORDING' just because images
        # flow, while every write silently fails with ENODEV. Latched.
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        self._storage_status = ''
        _st_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._storage_status_sub = self.create_subscription(
            String, '/Multiespectral/recording_status', self._storage_status_cb, _st_qos)

        # ---- Camera subscriptions (ImageWithMetadata) ----
        self._cam1_sub = self.create_subscription(
            ImageWithMetadata, camera1_topic, self._camera1_cb, 1)
        self._cam2_sub = self.create_subscription(
            ImageWithMetadata, camera2_topic, self._camera2_cb, 1)

        # ---- LIDAR subscription (sensor_msgs/Image) ----
        self._lidar_sub = None
        self._lidar_available = self._check_topic_exists(lidar_topic)
        if self._lidar_available:
            self._lidar_sub = self.create_subscription(
                Image, lidar_topic, self._lidar_cb, 1)
            self._freq_lidar.start()
            self.get_logger().info(f"LIDAR topic '{lidar_topic}' found.")
        else:
            self.get_logger().warning(
                f"LIDAR topic '{lidar_topic}' not found. LIDAR display disabled.")

        # ---- SocketIO update thread ----
        self._update_thread = threading.Thread(target=self._update_loop, daemon=True)
        self._update_thread.start()
        self.get_logger().info("Node initialized.")

    # -------------------------------------------------------------------------
    # Recording control
    # -------------------------------------------------------------------------

    def set_recording(self, enabled: bool):
        if enabled and not self._recording_enabled:
            self._saved_cam1 = 0
            self._saved_cam2 = 0
            self._saved_lidar = 0
        self._recording_enabled = enabled
        msg = Bool()
        msg.data = enabled
        self._recording_pub.publish(msg)
        self.get_logger().info(f'Recording {"ENABLED" if enabled else "DISABLED"}')

    def _storage_status_cb(self, msg):
        self._storage_status = msg.data

    def _recording_status_cb(self, msg):
        if msg.data != self._recording_enabled:
            self._recording_enabled = msg.data
            self.get_logger().info(
                f'Recording state synced from external: {"ENABLED" if msg.data else "DISABLED"}')

    def enable_recording(self):   self.set_recording(True)
    def disable_recording(self):  self.set_recording(False)
    def toggle_recording(self):   self.set_recording(not self._recording_enabled)

    # -------------------------------------------------------------------------
    # Topic helpers
    # -------------------------------------------------------------------------

    def _check_topic_exists(self, topic_name):
        if not topic_name:
            return False
        try:
            names = {t for t, _ in self.get_topic_names_and_types()}
            normalized = topic_name if topic_name.startswith('/') else '/' + topic_name
            if normalized in names:
                return True
            base = topic_name.split('/')[-1]
            return any(t.endswith('/' + base) for t in names)
        except Exception:
            return False

    def _recheck_lidar_availability(self):
        if not self.current_lidar_topic:
            return
        exists = self._check_topic_exists(self.current_lidar_topic)
        if exists and not self._lidar_available:
            self.get_logger().info(
                f"LIDAR topic '{self.current_lidar_topic}' now available. Subscribing...")
            if self._lidar_sub:
                self.destroy_subscription(self._lidar_sub)
            self._lidar_sub = self.create_subscription(
                Image, self.current_lidar_topic, self._lidar_cb, 1)
            self._lidar_available = True
            self._freq_lidar.start()
        elif not exists and self._lidar_available:
            self.get_logger().warning(
                f"LIDAR topic '{self.current_lidar_topic}' no longer available.")
            if self._lidar_sub:
                self.destroy_subscription(self._lidar_sub)
                self._lidar_sub = None
            self._lidar_available = False

    def change_lidar_topic(self, data):
        new_topic = data.get('topic', '')
        if not new_topic or new_topic == self.current_lidar_topic:
            return
        self.get_logger().info(
            f"Changing LIDAR topic: '{self.current_lidar_topic}' -> '{new_topic}'")
        if self._lidar_sub:
            self.destroy_subscription(self._lidar_sub)
            self._lidar_sub = None
        self.current_lidar_topic = new_topic
        with self._frame_lock:
            self._frames['lidar'] = None   # drop stale frame from previous topic
        self._lidar_sub = self.create_subscription(
            Image, new_topic, self._lidar_cb, 1)
        self._lidar_available = True
        self._freq_lidar.start()
        self.get_logger().info(f"Subscribed to LIDAR: {new_topic}")

    def update_image_size(self, size_data):
        for key_new, key_old in [('camera1', 'lwir'), ('camera2', 'rgb'), ('lidar', 'swir')]:
            src = size_data.get(key_new) or size_data.get(key_old)
            if src:
                self._image_size[key_new] = (int(src['width']), int(src['height']))

    # -------------------------------------------------------------------------
    # MJPEG frame store (served by Flask /stream/<name>, not over Socket.IO)
    # -------------------------------------------------------------------------

    def _set_frame(self, key, jpg_bytes):
        with self._frame_lock:
            self._frames[key] = jpg_bytes
            self._frame_seq[key] += 1

    def get_frame(self, key):
        """Return (latest_jpeg_bytes_or_None, seq) for the MJPEG stream `key`."""
        with self._frame_lock:
            return self._frames.get(key), self._frame_seq.get(key, 0)

    # -------------------------------------------------------------------------
    # Image callbacks
    # -------------------------------------------------------------------------

    def _camera1_cb(self, msg: ImageWithMetadata):
        image = self._convert_image_with_metadata(msg)
        if image is not None:
            resized = cv2.resize(image, self._image_size['camera1'])
            ok, buf = cv2.imencode('.jpg', resized, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                self._set_frame('camera1', buf.tobytes())
            self._freq_cam1.tick()
            if self._recording_enabled:
                self._saved_cam1 += 1
        else:
            self._throttle_warn('cam1', 10.0, "Failed to convert camera1 image.")

    def _camera2_cb(self, msg: ImageWithMetadata):
        image = self._convert_image_with_metadata(msg)
        if image is not None:
            resized = cv2.resize(image, self._image_size['camera2'])
            ok, buf = cv2.imencode('.jpg', resized, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                self._set_frame('camera2', buf.tobytes())
            self._freq_cam2.tick()
            if self._recording_enabled:
                self._saved_cam2 += 1
        else:
            self._throttle_warn('cam2', 10.0, "Failed to convert camera2 image.")

    def _lidar_cb(self, msg: Image):
        image = self._convert_ros_image(msg)
        if image is not None:
            resized = cv2.resize(image, self._image_size['lidar'])
            ok, buf = cv2.imencode('.jpg', resized, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                self._set_frame('lidar', buf.tobytes())
            self._freq_lidar.tick()
            if self._recording_enabled:
                self._saved_lidar += 1
        else:
            self._throttle_warn('lidar', 10.0,
                f"Failed to convert LIDAR image from {self.current_lidar_topic}")

    # -------------------------------------------------------------------------
    # Image conversion helpers
    # -------------------------------------------------------------------------

    def _convert_image_with_metadata(self, msg: ImageWithMetadata):
        return self._convert_ros_image(msg.image)

    def _convert_ros_image(self, ros_image):
        try:
            if isinstance(ros_image, CompressedImage):
                np_arr = np.frombuffer(ros_image.data, np.uint8)
                if hasattr(ros_image, 'format') and 'mono8' in ros_image.format:
                    return cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)
                return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            enc = ros_image.encoding
            if enc == 'mono8':
                return self._bridge.imgmsg_to_cv2(ros_image, 'mono8')
            elif enc == 'mono16':
                img16 = self._bridge.imgmsg_to_cv2(ros_image, 'mono16')
                return cv2.normalize(img16, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            elif enc == 'bgr8':
                return self._bridge.imgmsg_to_cv2(ros_image, 'bgr8')
            elif enc == 'rgb8':
                img = self._bridge.imgmsg_to_cv2(ros_image, 'rgb8')
                return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            elif enc.startswith('32FC'):
                imgf = self._bridge.imgmsg_to_cv2(ros_image, enc)
                return cv2.normalize(imgf, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            else:
                img = self._bridge.imgmsg_to_cv2(ros_image, desired_encoding='passthrough')
                if img.dtype in (np.uint16, np.float32):
                    img = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                if len(img.shape) > 2 and img.shape[2] > 1:
                    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                return img
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return None

    # -------------------------------------------------------------------------
    # SocketIO update loop
    # -------------------------------------------------------------------------

    def _update_loop(self):
        prev_c1, prev_c2, prev_li = 0, 0, 0
        recheck_cycle = 0

        while self._running and rclpy.ok():
            recheck_cycle += 1
            if recheck_cycle % 5 == 0:
                self._recheck_lidar_availability()

            c1 = self._freq_cam1.countItems()
            c2 = self._freq_cam2.countItems()
            li = self._freq_lidar.countItems()

            images_flowing = (c1 > prev_c1) or (c2 > prev_c2)
            if self._lidar_available:
                images_flowing = images_flowing or (li > prev_li)

            # NO_DISK from the compositor overrides everything: images may be
            # flowing but NOTHING is being written. Make that unmissable.
            storage_ok = (self._storage_status != 'NO_DISK')
            if self._recording_enabled and not storage_ok:
                status = '⚠ NO GRABA — disco no montado'
            elif self._recording_enabled and images_flowing:
                status = 'RECORDING'
            elif self._recording_enabled:
                status = 'RECORDING REQUESTED'
            elif images_flowing:
                status = 'READY'
            else:
                status = 'IDLE'

            data = {
                'total_images_received_camera1': c1,
                'total_images_received_camera2': c2,
                'saved_frames_camera1':  self._saved_cam1,
                'saved_frames_camera2':  self._saved_cam2,
                'recording_enabled':   self._recording_enabled,
                'recording_status':    status,
                'storage_ok':          storage_ok,
                'storage_status':      self._storage_status,
                'frame_rate_camera1':  str(self._freq_cam1),
                'frame_rate_camera2':  str(self._freq_cam2),
                'lidar_available':     self._lidar_available,
                'camera1_name':        self.camera1_display_name,
                'camera2_name':        self.camera2_display_name,
                'lidar_name':          self.lidar_display_name,
                'gui_title':           self.gui_title,
                'lidar_topic_options': self.lidar_topic_options,
                'current_lidar_topic': self.current_lidar_topic,
            }
            if self._lidar_available:
                data['total_images_received_lidar'] = li
                data['saved_frames_lidar'] = self._saved_lidar
                data['frame_rate_lidar'] = str(self._freq_lidar)

            self.socketio.emit('update_data', data)
            prev_c1, prev_c2, prev_li = c1, c2, li
            time.sleep(1)

    def stop(self):
        self.get_logger().info("Stopping node.")
        self._running = False
        self.set_recording(False)
        if self._update_thread:
            self._update_thread.join(timeout=2.0)

    # -------------------------------------------------------------------------

    def _throttle_warn(self, key, interval_sec, msg):
        now = time.monotonic()
        if now - self._throttle_times.get(key, 0) >= interval_sec:
            self._throttle_times[key] = now
            self.get_logger().warning(msg)
