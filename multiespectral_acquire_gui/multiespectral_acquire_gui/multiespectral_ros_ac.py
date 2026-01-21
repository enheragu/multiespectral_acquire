#!/usr/bin/env python3
# encoding: utf-8

import time
import cv2
import base64
import threading
import numpy as np

import rospy
import actionlib
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

from multiespectral_acquire.msg import MultiespectralAcquisitionAction, MultiespectralAcquisitionGoal, MultiespectralAcquisitionFeedback
from multiespectral_acquire_gui.FreqCounter import FreqCounter

frame_rate_lwir = FreqCounter()
frame_rate_rgb = FreqCounter()
frame_rate_lidar = FreqCounter()
lwir_img_path = ""
rgb_img_path = ""
lidar_img_path = ""
storage_path = "not-updated-yet"
images_acquired = 0
store_in_drive = False
camera_handler = None
lidar_available = False

basler_ac_name = "AS"
flir_topic_name = "lwir_camera/compressed"
basler_topic_name = "visible_camera/compressed"
lidar_topic_name = "swir_lidar/compressed"
image_size = {'lwir': (320, 240), 'rgb': (320, 240), 'swir': (320, 240)}

bridge = CvBridge()

class RosMultiespectralAcquire(object):
    def __init__(self, socketio):
        global lidar_available
        self.socketio = socketio
        self.client = actionlib.SimpleActionClient(basler_ac_name, MultiespectralAcquisitionAction)
        rospy.loginfo(f'Wait for "{basler_ac_name}" action server')
        self.client.wait_for_server()

        self.goal_handle = None
        self._running = True
        
        # Camera subscribers (always)
        self.image_sub1 = rospy.Subscriber(flir_topic_name, CompressedImage, self.lwir_image_cb, queue_size=1)
        self.image_sub2 = rospy.Subscriber(basler_topic_name, CompressedImage, self.rgb_image_cb, queue_size=1)
        
        # LiDAR subscriber (optional - check if topic exists)
        self.lidar_sub = None
        lidar_available = self._check_topic_exists(lidar_topic_name, timeout=2.0)
        if lidar_available:
            self.lidar_sub = rospy.Subscriber(lidar_topic_name, Image, self.lidar_image_cb, queue_size=1)
            rospy.loginfo(f"[MultiespectralAcquireGui] LiDAR topic '{lidar_topic_name}' found.")
        else:
            rospy.logwarn(f"[MultiespectralAcquireGui] LiDAR topic '{lidar_topic_name}' not found. LIDAR display disabled.")
        
        self.update_socketio_thread = threading.Thread(target=self.updateSocketio, daemon=True)
        self.update_socketio_thread.start()
        self.socketio.on('image_size', self.update_image_size)
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
        if self.update_socketio_thread:
            self.update_socketio_thread.join(timeout=2.0)

    def sendGoal(self, store: bool):
        if self.client.simple_state == actionlib.SimpleGoalState.ACTIVE:
            rospy.loginfo('[sendGoal] Canceling previous goal.')
            self.client.cancel_goal()
        goal = MultiespectralAcquisitionGoal()
        goal.store = store

        frame_rate_lwir.start()
        frame_rate_rgb.start()
        if lidar_available:
            frame_rate_lidar.start()
        self.client.send_goal(goal, feedback_cb=self.feedback_cb, done_cb=self.result_callback)
        return True

    def result_callback(self, status, result):
        global images_acquired, storage_path
        rospy.loginfo(f'[result_callback] Goal FINISHED: {result}.')
        self.goal_handle = None
        frame_rate_lwir.stop()
        frame_rate_rgb.stop()
        frame_rate_lidar.stop()
        if hasattr(result, 'images_acquired'):
            images_acquired = result.images_acquired
        if hasattr(result, 'storage_path'):
            storage_path = result.storage_path

    def feedback_cb(self, feedback):
        global storage_path, images_acquired
        if hasattr(feedback, 'storage_path'):
            storage_path = feedback.storage_path
        if hasattr(feedback, 'images_acquired'):
            images_acquired = feedback.images_acquired

    def cancelGoal(self):
        if self.client.simple_state != actionlib.SimpleGoalState.ACTIVE:
            rospy.logwarn('[cancelGoal] No active goal to cancel.')
            return
        rospy.loginfo('[cancelGoal] Canceling goal.')
        self.client.cancel_goal()

    def update_image_size(self, size_data):
        global image_size
        image_size['lwir'] = (size_data['lwir']['width'], size_data['lwir']['height'])
        image_size['rgb'] = (size_data['rgb']['width'], size_data['rgb']['height'])
        if 'swir' in size_data:
            image_size['swir'] = (size_data['swir']['width'], size_data['swir']['height'])

    def lwir_image_cb(self, msg):
        global lwir_img_path
        image = self.convert_image(msg)
        if image is not None:
            resized_image = cv2.resize(image, image_size['lwir'])
            _, lwir_buffer = cv2.imencode('.png', resized_image)
            lwir_img_path = base64.b64encode(lwir_buffer).decode('utf-8')
            frame_rate_lwir.tick()
        else:
            rospy.logwarn("[lwir_image_cb] Failed to convert LWIR image.")

    def rgb_image_cb(self, msg):
        global rgb_img_path
        image = self.convert_image(msg)
        if image is not None:
            resized_image = cv2.resize(image, image_size['rgb'])
            _, rgb_buffer = cv2.imencode('.png', resized_image)
            rgb_img_path = base64.b64encode(rgb_buffer).decode('utf-8')
            frame_rate_rgb.tick()
        else:
            rospy.logwarn("[rgb_image_cb] Failed to convert RGB image.")

    def lidar_image_cb(self, msg):
        global lidar_img_path
        image = self.convert_image(msg)
        if image is not None:
            resized_image = cv2.resize(image, image_size['swir'])
            _, lidar_buffer = cv2.imencode('.png', resized_image)
            lidar_img_path = base64.b64encode(lidar_buffer).decode('utf-8')
            frame_rate_lidar.tick()

    def updateSocketio(self):
        while self._running and not rospy.is_shutdown():
            data = {
                'total_images_received_lwir': frame_rate_lwir.countItems(),
                'total_images_received_rgb': frame_rate_rgb.countItems(),
                'lwir_img_path': lwir_img_path,
                'rgb_img_path': rgb_img_path,
                'storage_path': storage_path,
                'frame_rate_lwir': str(frame_rate_lwir),
                'frame_rate_rgb': str(frame_rate_rgb),
                'lidar_available': lidar_available,
            }
            if lidar_available:
                data['total_images_received_lidar'] = frame_rate_lidar.countItems()
                data['lidar_img_path'] = lidar_img_path
                data['frame_rate_lidar'] = str(frame_rate_lidar)
            self.socketio.emit('update_data', data)
            time.sleep(1)

    def convert_image(self, ros_image):
        try:
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
                rospy.logwarn("[convert_image] Received empty image message.")
                return None
        except CvBridgeError as e:
            rospy.logerr(f'[convert_image] CvBridge Error: {e}')
            return None