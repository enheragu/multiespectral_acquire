
#!/usr/bin/env python3
# encoding: utf-8


import time
import cv2
import base64
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image, CompressedImage

from cv_bridge import CvBridge, CvBridgeError

from multiespectral_acquire.action import MultiespectralAcquisition  # Action
from multiespectral_acquire_gui.FreqCounter import FreqCounter

frame_rate_lwir = FreqCounter()
frame_rate_rgb = FreqCounter()
lwir_img_path = ""
rgb_img_path = ""
lwir_img_storepath = ""
rgb_img_storepath = ""
store_in_drive = False
camera_handler = None

# flir_ac_name = "MultiespectralAcquire_lwir"
basler_ac_name = "AS"
flir_topic_name = "lwir_camera/compressed"
basler_topic_name = "visible_camera/compressed"
image_size = {'lwir': (320, 240), 'rgb': (320, 240)}

bridge = CvBridge()


# Which actions will be used
ac_subs_list = [basler_ac_name]  # flir_ac_name,  # FLIR working as slave already :)

class RosMultiespectralAcquire(Node):
    def __init__(self, socketio):
        if not rclpy.ok():
            rclpy.init()

        super().__init__('multiespectral_flask_gui')
        self.update_socketio_thread = None

        self.running = False
        self.socketio = socketio
        
        self.client = []
        for ac_name in ac_subs_list:

            self.client.append(ActionClient(self, MultiespectralAcquisition, ac_name))
            self.get_logger().info(f'Wait for "{ac_name}" server')
            self.client[-1].wait_for_server()

        # Suscribirse a los topics de imagen
        self.image_sub1 = self.create_subscription(CompressedImage, flir_topic_name, self.lwir_image_cb, 10)
        self.image_sub2 = self.create_subscription(CompressedImage, basler_topic_name, self.rgb_image_cb, 10)
        
        try:
            self.update_socketio_thread = threading.Thread(target=self.updateSocketio)
            self.update_socketio_thread.start()
            self.socketio.on('image_size', self.update_image_size)
        except Exception as e:
            self.get_logger().error(f"Error initializing socket: {e}")
            raise e

         
    def __del__(self):
        self.shutdown()
        rclpy.shutdown()
    
    def shutdown(self):        
        self.get_logger().info("[MultiespectralAcquireGui] Destructor")
        if self.update_socketio_thread:
            self.update_socketio_thread.join(timeout=2.0)
        
    def cancelGoal(self):
        for client in self.client:
            client.cancel_goal()

    def stop(self):
        self.get_logger().info("Stopping image acquisition")
        self.cancelGoal()
        frame_rate_lwir.stop()
        frame_rate_rgb.stop()
        self.get_logger().info("Finished image acquisition")

    def sendGoal(self, store):
        frame_rate_lwir.start()
        frame_rate_rgb.start()

        goal = MultiespectralAcquisition.Goal()
        goal.store = store

        self.get_logger().info(f'Send goal with store flag as {store}.')
        for client in self.client:
            future = client.send_goal_async(goal, feedback_callback=self.feedback_cb)
            future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Finished goal: {result}')

    def feedback_cb(self, goal_handle, feedback):
        global lwir_img_storepath, rgb_img_storepath
        # self.get_logger().info(f'Feedback: Images Acquired = {feedback.images_acquired}, Storage Path = {feedback.storage_path}')
        if 'lwir' in feedback.feedback.storage_path:
            lwir_img_storepath = feedback.feedback.storage_path
        else:
            rgb_img_storepath = feedback.feedback.storage_path

    def update_image_size(self, size_data):
        global image_size
        image_size['lwir'] = (size_data['lwir']['width'], size_data['lwir']['height'])
        image_size['rgb'] = (size_data['rgb']['width'], size_data['rgb']['height']) 

    def lwir_image_cb(self, msg):
        global lwir_img_path, total_images_received_lwir
        image = self.convert_image(msg)
        self.get_logger().info("[lwir_image_cb] Called")
        if image is not None:
            self.get_logger().info("Got new LWIR Image.")
            resized_image = cv2.resize(image, image_size['lwir'])
            filtered_image = cv2.bilateralFilter(resized_image, d=9, sigmaColor=75, sigmaSpace=75)
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            clahe_image = clahe.apply(filtered_image)
            _, lwir_buffer = cv2.imencode('.png', clahe_image)
            lwir_img_path = base64.b64encode(lwir_buffer).decode('utf-8')
            frame_rate_lwir.tick()
        else:
            self.get_logger().warn("Failed to convert LWIR image.")

    def rgb_image_cb(self, msg):
        global rgb_img_path, total_images_received_rgb
        image = self.convert_image(msg)
        self.get_logger().info("[rgb_image_cb] Called")
        if image is not None:
            self.get_logger().info("Got new RGB Image.")
            resized_image = cv2.resize(image, image_size['rgb'])
            _, rgb_buffer = cv2.imencode('.png', resized_image)
            rgb_img_path = base64.b64encode(rgb_buffer).decode('utf-8')
            frame_rate_rgb.tick()
        else:
            self.get_logger().warn("Failed to convert RGB image.")

    def updateSocketio(self):
        while True:
            self.socketio.emit('update_data', {
                    'total_images_received_lwir': frame_rate_lwir.cuontItems(),
                    'total_images_received_rgb': frame_rate_rgb.cuontItems(),
                    'lwir_img_path': lwir_img_path,
                    'rgb_img_path': rgb_img_path,
                    'lwir_img_storepath': lwir_img_storepath,
                    'rgb_img_storepath': rgb_img_storepath,
                    'frame_rate_lwir': str(frame_rate_lwir),
                    'frame_rate_rgb': str(frame_rate_rgb)
                })
            time.sleep(1)
        
    def convert_image(self, ros_image):
        try:
            if ros_image.data and isinstance(ros_image, Image):
                if ros_image.encoding == "mono8":
                    cv_image = bridge.imgmsg_to_cv2(ros_image, "mono8")
                elif ros_image.encoding == "bgr8":
                    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
                else:
                    self.get_logger().warn(f"Unexpected image encoding: {ros_image.encoding}. Defaulting to 'bgr8'.")
                    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
                return cv_image
            elif isinstance(ros_image, CompressedImage):
                np_arr = np.frombuffer(ros_image.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE if "mono8" in ros_image.format else cv2.IMREAD_COLOR)
                return cv_image
            else:
                self.get_logger().warn("Received empty image message.")
                return None
        except CvBridgeError as e:
            self.get_logger().err(f'CvBridge Error: {e}')
            return None