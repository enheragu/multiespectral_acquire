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

# LIDAR topic options (will be read from parameters)
lidar_topic_options = []

image_size = {'camera1': (320, 240), 'camera2': (320, 240), 'lidar': (320, 240)}

bridge = CvBridge()

class RosMultiespectralAcquire(object):
    def __init__(self, socketio):
        global lidar_available, camera1_display_name, camera2_display_name, lidar_display_name, gui_title, lidar_topic_options
        self.socketio = socketio
        
        # Read GUI configuration from ROS parameters
        gui_title = rospy.get_param('~gui_title', 'Camera GUI')
        
        # Read camera configuration from ROS parameters
        camera1_topic = rospy.get_param('~camera1_topic', '/Multiespectral/lwir_camera/image_with_metadata_sync')
        camera2_topic = rospy.get_param('~camera2_topic', '/Multiespectral/visible_camera/image_with_metadata_sync')
        lidar_topic = rospy.get_param('~lidar_topic', '/Multiespectral/ouster/reflec_image_sync_cropped_sync')
        
        camera1_display_name = rospy.get_param('~camera1_name', 'Camera 1')
        camera2_display_name = rospy.get_param('~camera2_name', 'Camera 2')
        lidar_display_name = rospy.get_param('~lidar_name', 'LIDAR Image')
        
        # Read LIDAR topic options from ROS parameters
        lidar_topic_options = rospy.get_param('~lidar_topic_options', [
            {'topic': '/Multiespectral/ouster/range_image_sync_cropped_sync', 'label': 'Range Image'},
            {'topic': '/Multiespectral/ouster/reflec_image_sync_cropped_sync', 'label': 'Reflectivity Image'},
            {'topic': '/Multiespectral/ouster/signal_image_sync_cropped_sync', 'label': 'Signal Image'},
            {'topic': '/Multiespectral/ouster/nearir_image_sync_cropped_sync', 'label': 'Near-IR Image'}
        ])
        
        rospy.loginfo(f'[MultiespectralAcquireGui] GUI Title: "{gui_title}"')
        rospy.loginfo(f'[MultiespectralAcquireGui] Camera 1: "{camera1_display_name}" -> {camera1_topic}')
        rospy.loginfo(f'[MultiespectralAcquireGui] Camera 2: "{camera2_display_name}" -> {camera2_topic}')
        rospy.loginfo(f'[MultiespectralAcquireGui] LIDAR: "{lidar_display_name}" -> {lidar_topic}')
        
        # Publisher for recording control
        self.recording_pub = rospy.Publisher(recording_control_topic, Bool, queue_size=1, latch=True)
        rospy.loginfo(f'[MultiespectralAcquireGui] Publishing recording control to: {recording_control_topic}')
        
        # Subscriber to sync recording state from other sources (other GUIs, etc.)
        self.recording_sub = rospy.Subscriber(recording_control_topic, Bool, self._recording_status_cb, queue_size=1)
        rospy.loginfo(f'[MultiespectralAcquireGui] Subscribed to recording control for sync: {recording_control_topic}')

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
        
        # Start frequency counters immediately (not just when recording)
        frame_rate_camera1.start()
        frame_rate_camera2.start()
        
        # LiDAR subscriber (optional - check if topic exists)
        self.lidar_sub = None
        self.current_lidar_topic = lidar_topic
        lidar_available = self._check_topic_exists(lidar_topic, timeout=2.0)
        if lidar_available:
            self.lidar_sub = rospy.Subscriber(lidar_topic, rospy.AnyMsg, self.lidar_image_cb, queue_size=1)
            frame_rate_lidar.start()  # Start LIDAR counter if available
            rospy.loginfo(f"[MultiespectralAcquireGui] LiDAR topic '{lidar_topic}' found.")
        else:
            rospy.logwarn(f"[MultiespectralAcquireGui] LiDAR topic '{lidar_topic}' not found. LIDAR display disabled.")
        
        self.update_socketio_thread = threading.Thread(target=self.updateSocketio, daemon=True)
        self.update_socketio_thread.start()
        rospy.loginfo("[MultiespectralAcquireGui] Node initialized.")

    def _check_topic_exists(self, topic_name, timeout=2.0):
        """Check if a topic exists by looking at published topics"""
        if not topic_name:
            return False
        try:
            published_topics = rospy.get_published_topics()
            topic_names = [t[0] for t in published_topics]
            
            # Normalize topic name (ensure it has leading slash)
            normalized_topic = topic_name if topic_name.startswith('/') else '/' + topic_name
            
            # Direct match
            if normalized_topic in topic_names:
                return True
            
            # Check without namespace prefix if not found
            # E.g., if topic is "ouster/reflec_image_sync_cropped" try with current namespace
            if not topic_name.startswith('/'):
                ns = rospy.get_namespace().rstrip('/')
                full_topic = ns + '/' + topic_name if ns else '/' + topic_name
                if full_topic in topic_names:
                    return True
            
            # Partial match as last resort (topic might have different prefix)
            base_topic = topic_name.split('/')[-1]  # Get last part
            for t in topic_names:
                if t.endswith('/' + base_topic):
                    rospy.logdebug(f"[_check_topic_exists] Partial match: {topic_name} found as {t}")
                    return True
            
            return False
        except Exception as e:
            rospy.logdebug(f"[_check_topic_exists] Error checking topic: {e}")
            return False
    
    def _recheck_lidar_availability(self):
        """Periodically check if LIDAR topic becomes available or unavailable"""
        global lidar_available
        
        if not self.current_lidar_topic:
            return
        
        # Check if topic has active publishers
        try:
            topic_exists = self._check_topic_exists(self.current_lidar_topic, timeout=0.5)
        except Exception as e:
            rospy.logwarn_throttle(10.0, f"[_recheck_lidar_availability] Error checking topic: {e}")
            return
        
        rospy.loginfo_throttle(10.0, f"[_recheck_lidar_availability] '{self.current_lidar_topic}': exists={topic_exists}, available={lidar_available}")
        
        # If topic exists but we're not subscribed, subscribe
        if topic_exists and not lidar_available:
            rospy.loginfo(f"[_recheck_lidar_availability] LIDAR topic '{self.current_lidar_topic}' now available! Subscribing...")
            if self.lidar_sub is not None:
                self.lidar_sub.unregister()
            self.lidar_sub = rospy.Subscriber(self.current_lidar_topic, rospy.AnyMsg, self.lidar_image_cb, queue_size=1)
            lidar_available = True
            frame_rate_lidar.start()
        
        # If topic doesn't exist but we think it's available, mark as unavailable
        elif not topic_exists and lidar_available:
            rospy.logwarn(f"[_recheck_lidar_availability] LIDAR topic '{self.current_lidar_topic}' no longer available.")
            if self.lidar_sub is not None:
                self.lidar_sub.unregister()
                self.lidar_sub = None
            lidar_available = False

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
        return True

    def _recording_status_cb(self, msg):
        """Callback to sync recording state from other sources (other GUIs, command line, etc.)"""
        global recording_enabled
        if msg.data != recording_enabled:
            recording_enabled = msg.data
            rospy.loginfo(f'[_recording_status_cb] Recording state synced from external source: {"ENABLED" if msg.data else "DISABLED"}')

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
        # Convert to int explicitly for OpenCV compatibility
        if 'camera1' in size_data:
            image_size['camera1'] = (int(size_data['camera1']['width']), int(size_data['camera1']['height']))
        elif 'lwir' in size_data:
            image_size['camera1'] = (int(size_data['lwir']['width']), int(size_data['lwir']['height']))
            
        if 'camera2' in size_data:
            image_size['camera2'] = (int(size_data['camera2']['width']), int(size_data['camera2']['height']))
        elif 'rgb' in size_data:
            image_size['camera2'] = (int(size_data['rgb']['width']), int(size_data['rgb']['height']))
            
        if 'lidar' in size_data:
            image_size['lidar'] = (int(size_data['lidar']['width']), int(size_data['lidar']['height']))
        elif 'swir' in size_data:
            image_size['lidar'] = (int(size_data['swir']['width']), int(size_data['swir']['height']))

    def change_lidar_topic(self, data):
        """Change LIDAR subscription to a different topic"""
        global lidar_available
        
        new_topic = data.get('topic', '')
        if not new_topic:
            rospy.logwarn("[change_lidar_topic] No topic specified")
            return
        
        rospy.loginfo(f"[change_lidar_topic] Changing from '{self.current_lidar_topic}' to '{new_topic}'")
        
        # Unsubscribe from current topic
        if self.lidar_sub is not None:
            self.lidar_sub.unregister()
            self.lidar_sub = None
            rospy.sleep(0.1)
        
        # Check if new topic exists
        topic_exists = self._check_topic_exists(new_topic, timeout=1.0)
        rospy.loginfo(f"[change_lidar_topic] Topic '{new_topic}' exists: {topic_exists}")
        
        if topic_exists:
            self.lidar_sub = rospy.Subscriber(new_topic, rospy.AnyMsg, self.lidar_image_cb, queue_size=1)
            self.current_lidar_topic = new_topic
            lidar_available = True
            frame_rate_lidar.start()
            rospy.loginfo(f"[change_lidar_topic] Successfully subscribed to: {new_topic}")
        else:
            self.current_lidar_topic = new_topic
            lidar_available = False
            rospy.logwarn(f"[change_lidar_topic] Topic {new_topic} not found. Will retry automatically.")

    def camera1_image_cb(self, msg):
        global camera1_img_path
        image = self.convert_image(msg)
        if image is not None:
            rospy.loginfo_throttle(5.0, f"[camera1_image_cb] Converted image shape={image.shape}, dtype={image.dtype}, min={image.min()}, max={image.max()}")
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
            # IMG correction to filter horizontal stripes (LIDAR-specific)
            img_reflec = image.astype(np.float32)
            mean_row = np.mean(img_reflec, axis=1, keepdims=True)  # Media de cada fila (canal)
            img_clean = img_reflec.astype(float) - mean_row  # Resta el sesgo de calibración
            img_normalized = np.clip((img_clean - img_clean.min()) / (img_clean.max() - img_clean.min()) * 255, 0, 255).astype(np.uint8)
            img = img_normalized
            resized_image = cv2.resize(image, image_size['lidar'])
            _, lidar_buffer = cv2.imencode('.png', resized_image)
            lidar_img_path = base64.b64encode(lidar_buffer).decode('utf-8')
            frame_rate_lidar.tick()
        else:
            rospy.logwarn(f"[lidar_image_cb] Failed to convert LIDAR image from {self.current_lidar_topic}")

    def updateSocketio(self):
        prev_count_camera1 = 0
        prev_count_camera2 = 0
        prev_count_lidar = 0
        recheck_cycle = 0
        
        while self._running and not rospy.is_shutdown():
            # Re-check LIDAR availability every 5 seconds
            recheck_cycle += 1
            if recheck_cycle % 5 == 0:
                self._recheck_lidar_availability()
            
            current_count_camera1 = frame_rate_camera1.countItems()
            current_count_camera2 = frame_rate_camera2.countItems()
            current_count_lidar = frame_rate_lidar.countItems()
            
            # Detect if images are being received
            images_flowing = (current_count_camera1 > prev_count_camera1) or (current_count_camera2 > prev_count_camera2)
            if lidar_available:
                images_flowing = images_flowing or (current_count_lidar > prev_count_lidar)
            
            # Determine system status
            if recording_enabled and images_flowing:
                status = 'RECORDING'
            elif recording_enabled and not images_flowing:
                status = 'RECORDING REQUESTED'
            elif not recording_enabled and images_flowing:
                status = 'STOP REQUESTED'
            else:
                status = 'IDLE'
            
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
                'lidar_topic_options': lidar_topic_options,
                'current_lidar_topic': self.current_lidar_topic,
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
                
                # Handle different encodings
                if ros_image.encoding == "mono8":
                    cv_image = bridge.imgmsg_to_cv2(ros_image, "mono8")
                elif ros_image.encoding == "mono16":
                    # Convert 16-bit to 8-bit for display (normalize to 0-255)
                    cv_image_16 = bridge.imgmsg_to_cv2(ros_image, "mono16")
                    # Normalize to 0-255 range for better visualization
                    cv_image = cv2.normalize(cv_image_16, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                elif ros_image.encoding == "bgr8":
                    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
                elif ros_image.encoding == "rgb8":
                    cv_image = bridge.imgmsg_to_cv2(ros_image, "rgb8")
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
                elif ros_image.encoding in ["32FC1", "32FC2", "32FC3"]:
                    # Float images (e.g., depth, range)
                    cv_image_float = bridge.imgmsg_to_cv2(ros_image, ros_image.encoding)
                    # Normalize float to 0-255 range
                    cv_image = cv2.normalize(cv_image_float, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                    if len(cv_image_float.shape) == 2:  # Single channel
                        pass  # Already grayscale
                    elif cv_image_float.shape[2] == 1:
                        cv_image = cv_image[:, :, 0]  # Extract single channel
                else:
                    rospy.logwarn(f"[convert_image] Unexpected image encoding: {ros_image.encoding}. Attempting automatic conversion...")
                    try:
                        # Try passthrough conversion
                        cv_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")
                        # If it's multi-channel or 16-bit, convert to displayable format
                        if len(cv_image.shape) > 2 and cv_image.shape[2] > 1:
                            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY) if cv_image.shape[2] == 3 else cv_image[:,:,0]
                        if cv_image.dtype == np.uint16 or cv_image.dtype == np.float32:
                            cv_image = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                    except Exception as e:
                        rospy.logerr(f"[convert_image] Failed to convert image with encoding {ros_image.encoding}: {e}")
                        return None
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