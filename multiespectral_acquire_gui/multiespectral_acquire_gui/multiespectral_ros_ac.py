
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
storage_path = "not-updated-yet"
images_acquired = 0
store_in_drive = False
camera_handler = None

# flir_ac_name = "MultiespectralAcquire_lwir"
basler_ac_name = "AS"
flir_topic_name = "lwir_camera/compressed"
basler_topic_name = "visible_camera/compressed"
image_size = {'lwir': (320, 240), 'rgb': (320, 240)}

bridge = CvBridge()


class RosMultiespectralAcquire(Node):
    def __init__(self, socketio):
        super().__init__('multiespectral_flask_gui')
        self.socketio = socketio

        self.client = ActionClient(self, MultiespectralAcquisition, basler_ac_name)
        self.get_logger().info(f'Wait for "{basler_ac_name}" server')
        self.client.wait_for_server()

        # Solo un goal activo (si quieres varios, usa lista)
        self.goal_handle = None

        # Suscripciones
        self.image_sub1 = self.create_subscription(
            CompressedImage, flir_topic_name, self.lwir_image_cb, 10
        )
        self.image_sub2 = self.create_subscription(
            CompressedImage, basler_topic_name, self.rgb_image_cb, 10
        )

        # Hilo de SocketIO
        self._running = True
        self.update_socketio_thread = threading.Thread(target=self.updateSocketio, daemon=True)
        self.update_socketio_thread.start()
        self.socketio.on('image_size', self.update_image_size)

        self.get_logger().info("[MultiespectralAcquireGui] Node initialized.")

    def stop(self):
        self.get_logger().info("[stop] Destructor.")
        self._running = False
        if self.update_socketio_thread:
            self.update_socketio_thread.join(timeout=2.0)

    def sendGoal(self, store: bool):
        # Si hay un goal activo, cancélalo primero
        if self.goal_handle is not None:
            self.get_logger().info('[sendGoal] Canceling previous goal.')
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._cancel_previous_done)

        goal = MultiespectralAcquisition.Goal()
        goal.store = store

        frame_rate_lwir.start()
        frame_rate_rgb.start()

        future = self.client.send_goal_async(goal, feedback_callback=self.feedback_cb)
        future.add_done_callback(self.goal_response_callback)
        return True

    def _cancel_previous_done(self, future):
        resp = future.result()
        if len(resp.goals_canceling) > 0:
            self.get_logger().info('[_cancel_previous_done] Previous goal cancelled.')
        else:
            self.get_logger().warn('[_cancel_previous_done] Previous goal was not cancelled.')


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('[goal_response_callback] Goal REJECTED by server!')
            return

        self.get_logger().info('[goal_response_callback] Goal ACCEPTED.')
        self.goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'[result_callback] Goal FINISHED: {result}.')
        self.goal_handle = None
        frame_rate_lwir.stop()
        frame_rate_rgb.stop()

    def feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        
        global storage_path, images_acquired
        storage_path = feedback.storage_path
        images_acquired = feedback.images_acquired

    def cancelGoal(self):
        if self.goal_handle is None:
            self.get_logger().warn('[cancelGoal] No active goal to cancel.')
            return

        self.get_logger().info('[cancelGoal] Canceling goal.')
        cancel_future = self.goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self._cancel_response_callback)

    def _cancel_response_callback(self, future):
        response = future.result()
        if len(response.goals_canceling) > 0:
            self.get_logger().info('[_cancel_response_callback] Goal cancelled successfully.')
        else:
            self.get_logger().warn('[_cancel_response_callback] Failed to cancel goal.')


    def update_image_size(self, size_data):
        global image_size
        image_size['lwir'] = (size_data['lwir']['width'], size_data['lwir']['height'])
        image_size['rgb'] = (size_data['rgb']['width'], size_data['rgb']['height']) 

    def lwir_image_cb(self, msg):
        global lwir_img_path, total_images_received_lwir
        image = self.convert_image(msg)
        if image is not None:
            # self.get_logger().info("Got new LWIR Image.")
            resized_image = cv2.resize(image, image_size['lwir'])
            # filtered_image = cv2.bilateralFilter(resized_image, d=9, sigmaColor=75, sigmaSpace=75)
            # if len(filtered_image.shape) == 3:  # Si bilateralFilter lo convirtió
            #     filtered_image = cv2.cvtColor(filtered_image, cv2.COLOR_BGR2GRAY)
            # clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            # resized_image = clahe.apply(filtered_image)
            _, lwir_buffer = cv2.imencode('.png', resized_image)
            lwir_img_path = base64.b64encode(lwir_buffer).decode('utf-8')
            frame_rate_lwir.tick()
        else:
            self.get_logger().warn("[lwir_image_cb] Failed to convert LWIR image.")

    def rgb_image_cb(self, msg):
        global rgb_img_path, total_images_received_rgb
        image = self.convert_image(msg)
        if image is not None:
            # self.get_logger().info("Got new RGB Image.")
            resized_image = cv2.resize(image, image_size['rgb'])
            _, rgb_buffer = cv2.imencode('.png', resized_image)
            rgb_img_path = base64.b64encode(rgb_buffer).decode('utf-8')
            frame_rate_rgb.tick()
        else:
            self.get_logger().warn("[rgb_image_cb] Failed to convert RGB image.")

    def updateSocketio(self):
        while self._running and rclpy.ok():
            self.socketio.emit('update_data', {
                'total_images_received_lwir': frame_rate_lwir.countItems(),
                'total_images_received_rgb': frame_rate_rgb.countItems(),
                'lwir_img_path': lwir_img_path,
                'rgb_img_path': rgb_img_path,
                'storage_path': storage_path,
                # 'images_acquired': images_acquired,
                'frame_rate_lwir': str(frame_rate_lwir),
                'frame_rate_rgb': str(frame_rate_rgb),
            })
            time.sleep(1)
        
    def convert_image(self, ros_image):
        try:
            if ros_image.data and isinstance(ros_image, Image):
                self.get_logger().debug(f"[convert_image] Image as {ros_image.encoding = }")
                if ros_image.encoding == "mono8":
                    cv_image = bridge.imgmsg_to_cv2(ros_image, "mono8")
                elif ros_image.encoding == "bgr8":
                    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
                else:
                    self.get_logger().warn(f"[convert_image] Unexpected image encoding: {ros_image.encoding}. Defaulting to 'bgr8'.")
                    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
                return cv_image
            elif isinstance(ros_image, CompressedImage):
                np_arr = np.frombuffer(ros_image.data, np.uint8)
                self.get_logger().debug(f"[convert_image] CompressedImage as {ros_image.format = }")
                if "mono8" in ros_image.format:
                    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)
                else:
                    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                return cv_image
            else:
                self.get_logger().warn("[convert_image] Received empty image message.")
                return None
        except CvBridgeError as e:
            self.get_logger().error(f'[convert_image] CvBridge Error: {e}')
            return None