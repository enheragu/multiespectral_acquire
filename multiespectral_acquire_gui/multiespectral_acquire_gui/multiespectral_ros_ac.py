#!/usr/bin/env python3
# encoding: utf-8

import time
import cv2
import base64
import threading
import numpy as np

import rospy
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError

from multiespectral_acquire_gui.FreqCounter import FreqCounter

frame_rate_camera1 = FreqCounter()
frame_rate_camera2 = FreqCounter()
frame_rate_lidar = FreqCounter()
camera1_img_path = ""
camera2_img_path = ""
lidar_img_path = ""
recording_enabled = False
lidar_available = False

# Camera names for display (will be read from parameters)
camera1_display_name = "Camera 1"
camera2_display_name = "Camera 2"
lidar_display_name = "LIDAR Image"
gui_title = "Camera GUI"

# Topic names (will be read from parameters)
recording_control_topic = "/Multiespectral/recording_enabled"

image_size = {'camera1': (320, 240), 'camera2': (320, 240), 'lidar': (320, 240)}

bridge = CvBridge()

class RosMultiespectralAcquire(object):
    def __init__(self, socketio):
        global lidar_available, camera1_display_name, camera2_display_name, lidar_display_name, gui_title
        self.socketio = socketio
        
        # Read GUI configuration from ROS parameters
        gui_title = rospy.get_param('~gui_title', 'Camera GUI')
        
        # Read camera configuration from ROS parameters
        camera1_topic = rospy.get_param('~camera1_topic', '/Multiespectral/lwir_camera/image_with_metadata_sync')
        camera2_topic = rospy.get_param('~camera2_topic', '/Multiespectral/visible_camera/image_with_metadata_sync')
        lidar_topic = rospy.get_param('~lidar_topic', '/Multiespectral/ouster/reflec_image_cropped_sync')
        
        camera1_display_name = rospy.get_param('~camera1_name', 'Camera 1')
        camera2_display_name = rospy.get_param('~camera2_name', 'Camera 2')
        lidar_display_name = rospy.get_param('~lidar_name', 'LIDAR Image')
        
        rospy.loginfo(f'[MultiespectralAcquireGui] GUI Title: "{gui_title}"')
        rospy.loginfo(f'[MultiespectralAcquireGui] Camera 1: "{camera1_display_name}" -> {camera1_topic}')
        rospy.loginfo(f'[MultiespectralAcquireGui] Camera 2: "{camera2_display_name}" -> {camera2_topic}')
        rospy.loginfo(f'[MultiespectralAcquireGui] LIDAR: "{lidar_display_name}" -> {lidar_topic}')
        
        # Publisher for recording control
        self.recording_pub = rospy.Publisher(recording_control_topic, Bool, queue_size=1, latch=True)
        rospy.loginfo(f'[MultiespectralAcquireGui] Publishing recording control to: {recording_control_topic}')
        
        # Initialize recording as disabled
        self.set_recording(False)

        self._running = True
        
        # Import message type for _sync topics
        try:
            from multiespectral_acquire.msg import ImageWithMetadata
            self.ImageWithMetadata = ImageWithMetadata
        except ImportError:
            rospy.logerr("[MultiespectralAcquireGui] Failed to import ImageWithMetadata message")
            self.ImageWithMetadata = None
        
        # Camera subscribers (to _sync topics - synchronized at master rate)
        self.image_sub1 = rospy.Subscriber(camera1_topic, rospy.AnyMsg, self.camera1_image_cb, queue_size=1)
        self.image_sub2 = rospy.Subscriber(camera2_topic, rospy.AnyMsg, self.camera2_image_cb, queue_size=1)
        
        # LiDAR subscriber (optional - check if topic exists)
        self.lidar_sub = None
        self.current_lidar_topic = lidar_topic
        lidar_available = self._check_topic_exists(lidar_topic, timeout=2.0)
        if lidar_available:
            self.lidar_sub = rospy.Subscriber(lidar_topic, rospy.AnyMsg, self.lidar_image_cb, queue_size=1)
            rospy.loginfo(f"[MultiespectralAcquireGui] LiDAR topic '{lidar_topic}' found.")
        else:
            rospy.logwarn(f"[MultiespectralAcquireGui] LiDAR topic '{lidar_topic}' not found. LIDAR display disabled.")
        
        self.update_socketio_thread = threading.Thread(target=self.updateSocketio, daemon=True)
        self.update_socketio_thread.start()
        self.socketio.on('image_size', self.update_image_size)
        self.socketio.on('change_lidar_topic', self.change_lidar_topic)
        rospy.loginfo("[MultiespectralAcquireGui] Node initialized.")

    def _check_topic_exists(self, topic_name, timeout=2.0):
        """Check if a topic exists by looking at published topics"""
        try:
            published_topics = rospy.get_published_topics()
            topic_names = [t[0] for t in published_topics]
            # Check both with and without leading slash
            full_topic = topic_name if topic_name.startswith('/') else '/' + rospy.get_namespace().strip('/') + '/' + topic_name
            return topic_name in topic_names or full_topic in topic_names or any(topic_name in t for t in topic_names)
        except:
            return False

    def stop(self):
        rospy.loginfo("[stop] Destructor.")
        self._running = False
        # Disable recording on shutdown
        self.set_recording(False)
        if self.update_socketio_thread:
            self.update_socketio_thread.join(timeout=2.0)

    def set_recording(self, enabled: bool):
        """Enable/disable recording across all buffer handlers"""
        global recording_enabled
        recording_enabled = enabled
        msg = Bool()
        msg.data = enabled
        self.recording_pub.publish(msg)
        rospy.loginfo(f'[set_recording] Recording {"ENABLED" if enabled else "DISABLED"}')
        
        if enabled:
            frame_rate_camera1.start()
            frame_rate_camera2.start()
            if lidar_available:
                frame_rate_lidar.start()
        else:
            frame_rate_camera1.stop()
            frame_rate_camera2.stop()
            if lidar_available:
                frame_rate_lidar.stop()
        
        return True

    def toggle_recording(self):
        """Toggle recording state"""
        return self.set_recording(not recording_enabled)

    def enable_recording(self):
        """Start recording"""
        return self.set_recording(True)

    def disable_recording(self):
        """Stop recording"""
        return self.set_recording(False)

    def update_image_size(self, size_data):
        global image_size
        # Support both old naming (lwir/rgb/swir) and new generic naming (camera1/camera2/lidar)
        if 'camera1' in size_data:
            image_size['camera1'] = (size_data['camera1']['width'], size_data['camera1']['height'])
        elif 'lwir' in size_data:
            image_size['camera1'] = (size_data['lwir']['width'], size_data['lwir']['height'])
            
        if 'camera2' in size_data:
            image_size['camera2'] = (size_data['camera2']['width'], size_data['camera2']['height'])
        elif 'rgb' in size_data:
            image_size['camera2'] = (size_data['rgb']['width'], size_data['rgb']['height'])
            
        if 'lidar' in size_data:
            image_size['lidar'] = (size_data['lidar']['width'], size_data['lidar']['height'])
        elif 'swir' in size_data:
            image_size['lidar'] = (size_data['swir']['width'], size_data['swir']['height'])

    def change_lidar_topic(self, data):
        """Change LIDAR subscription to a different topic"""
        global lidar_available
        import sys
        print("\n" + "="*80, file=sys.stderr)
        print("[LIDAR TOPIC CHANGE] EVENTO RECIBIDO!", file=sys.stderr)
        print(f"[LIDAR TOPIC CHANGE] Data received: {data}", file=sys.stderr)
        print("="*80 + "\n", file=sys.stderr)
        sys.stderr.flush()
        
        new_topic = data.get('topic', '')
        
        if not new_topic:
            print("[LIDAR TOPIC CHANGE] ERROR: No topic specified", file=sys.stderr)
            sys.stderr.flush()
            rospy.logwarn("[change_lidar_topic] No topic specified")
            return
        
        print(f"[LIDAR TOPIC CHANGE] Changing from '{self.current_lidar_topic}' to '{new_topic}'", file=sys.stderr)
        sys.stderr.flush()
        rospy.loginfo(f"[change_lidar_topic] Request to change from '{self.current_lidar_topic}' to '{new_topic}'")
        
        # Unsubscribe from current topic
        if self.lidar_sub is not None:
            self.lidar_sub.unregister()
            self.lidar_sub = None
            rospy.loginfo(f"[change_lidar_topic] Unsubscribed from {self.current_lidar_topic}")
        
        # Check if new topic exists
        topic_exists = self._check_topic_exists(new_topic, timeout=1.0)
        rospy.loginfo(f"[change_lidar_topic] Topic '{new_topic}' exists: {topic_exists}")
        
        if topic_exists:
            self.lidar_sub = rospy.Subscriber(new_topic, rospy.AnyMsg, self.lidar_image_cb, queue_size=1)
            self.current_lidar_topic = new_topic
            lidar_available = True
            rospy.loginfo(f"[change_lidar_topic] ✓ Successfully subscribed to {new_topic}")
        else:
            lidar_available = False
            rospy.logwarn(f"[change_lidar_topic] ✗ Topic {new_topic} not found. Available topics:")
            try:
                published_topics = rospy.get_published_topics()
                ouster_topics = [t[0] for t in published_topics if 'ouster' in t[0]]
                for t in ouster_topics:
                    rospy.logwarn(f"  - {t}")
            except:
                pass

    def camera1_image_cb(self, msg):
        global camera1_img_path
        image = self.convert_image(msg)
        if image is not None:
            resized_image = cv2.resize(image, image_size['camera1'])
            _, camera1_buffer = cv2.imencode('.png', resized_image)
            camera1_img_path = base64.b64encode(camera1_buffer).decode('utf-8')
            frame_rate_camera1.tick()
        else:
            rospy.logwarn("[camera1_image_cb] Failed to convert camera1 image.")

    def camera2_image_cb(self, msg):
        global camera2_img_path
        image = self.convert_image(msg)
        if image is not None:
            resized_image = cv2.resize(image, image_size['camera2'])
            _, camera2_buffer = cv2.imencode('.png', resized_image)
            camera2_img_path = base64.b64encode(camera2_buffer).decode('utf-8')
            frame_rate_camera2.tick()
        else:
            rospy.logwarn("[camera2_image_cb] Failed to convert camera2 image.")

    def lidar_image_cb(self, msg):
        global lidar_img_path
        image = self.convert_image(msg)
        if image is not None:
            resized_image = cv2.resize(image, image_size['lidar'])
            _, lidar_buffer = cv2.imencode('.png', resized_image)
            lidar_img_path = base64.b64encode(lidar_buffer).decode('utf-8')
            frame_rate_lidar.tick()

    def updateSocketio(self):
        prev_count_camera1 = 0
        prev_count_camera2 = 0
        prev_count_lidar = 0
        
        while self._running and not rospy.is_shutdown():
            current_count_camera1 = frame_rate_camera1.countItems()
            current_count_camera2 = frame_rate_camera2.countItems()
            current_count_lidar = frame_rate_lidar.countItems()
            
            # Detect if images are actually being received (include LIDAR if available)
            images_flowing = (current_count_camera1 > prev_count_camera1) or (current_count_camera2 > prev_count_camera2)
            if lidar_available:
                images_flowing = images_flowing or (current_count_lidar > prev_count_lidar)
            
            # Determine system status
            if recording_enabled and images_flowing:
                status = 'RECORDING'  # Actually recording
            elif recording_enabled and not images_flowing:
                status = 'RECORDING REQUESTED'  # Waiting for data
            elif not recording_enabled and images_flowing:
                status = 'STOP REQUESTED'  # Data still arriving after stop
            else:
                status = 'IDLE'  # Not recording and no data
            
            data = {
                'total_images_received_camera1': current_count_camera1,
                'total_images_received_camera2': current_count_camera2,
                'camera1_img_path': camera1_img_path,
                'camera2_img_path': camera2_img_path,
                'recording_enabled': recording_enabled,
                'recording_status': status,
                'frame_rate_camera1': str(frame_rate_camera1),
                'frame_rate_camera2': str(frame_rate_camera2),
                'lidar_available': lidar_available,
                'camera1_name': camera1_display_name,
                'camera2_name': camera2_display_name,
                'lidar_name': lidar_display_name,
                'gui_title': gui_title,
            }
            if lidar_available:
                data['total_images_received_lidar'] = current_count_lidar
                data['lidar_img_path'] = lidar_img_path
                data['frame_rate_lidar'] = str(frame_rate_lidar)
            
            self.socketio.emit('update_data', data)
            
            # Update previous counts for next iteration
            prev_count_camera1 = current_count_camera1
            prev_count_camera2 = current_count_camera2
            prev_count_lidar = current_count_lidar
            
            time.sleep(1)

    def convert_image(self, ros_message):
        try:
            # First, check if it's an AnyMsg (from _sync topics)
            if hasattr(ros_message, '_buff'):
                # Try to deserialize as ImageWithMetadata first (cameras)
                if self.ImageWithMetadata is not None:
                    try:
                        img_with_metadata = self.ImageWithMetadata().deserialize(ros_message._buff)
                        ros_image = img_with_metadata.image
                    except:
                        # If that fails, try as plain Image (LIDAR)
                        try:
                            ros_image = Image().deserialize(ros_message._buff)
                        except Exception as e:
                            rospy.logwarn(f"[convert_image] Failed to deserialize message: {e}")
                            return None
                else:
                    # No ImageWithMetadata, try Image directly
                    try:
                        ros_image = Image().deserialize(ros_message._buff)
                    except Exception as e:
                        rospy.logwarn(f"[convert_image] Failed to deserialize Image: {e}")
                        return None
            else:
                ros_image = ros_message
            
            # Now convert the image
            if hasattr(ros_image, 'data') and isinstance(ros_image, Image):
                rospy.logdebug(f"[convert_image] Image as {ros_image.encoding = }")
                if ros_image.encoding == "mono8":
                    cv_image = bridge.imgmsg_to_cv2(ros_image, "mono8")
                elif ros_image.encoding == "bgr8":
                    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
                else:
                    rospy.logwarn(f"[convert_image] Unexpected image encoding: {ros_image.encoding}. Defaulting to 'bgr8'.")
                    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
                return cv_image
            elif isinstance(ros_image, CompressedImage):
                np_arr = np.frombuffer(ros_image.data, np.uint8)
                rospy.logdebug(f"[convert_image] CompressedImage as {ros_image.format = }")
                if hasattr(ros_image, 'format') and "mono8" in ros_image.format:
                    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)
                else:
                    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                return cv_image
            else:
                rospy.logwarn("[convert_image] Received empty or unknown image message.")
                return None
        except CvBridgeError as e:
            rospy.logerr(f'[convert_image] CvBridge Error: {e}')
            return None