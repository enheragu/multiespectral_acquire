

import os
import time
import cv2
import base64
import threading
from cv_bridge import CvBridge, CvBridgeError

import rospy
import actionlib
from sensor_msgs.msg import Image

import multiespectral_fb.msg   
from FreqCounter import FreqCounter

frame_rate_lwir = FreqCounter()
frame_rate_rgb = FreqCounter()
lwir_img_path = ""
rgb_img_path = ""
lwir_img_storepath = ""
rgb_img_storepath = ""
store_in_drive = False
camera_handler = None

flir_ac_name = "MultiespectralAcquire_lwir"
basler_ac_name = "MultiespectralAcquire_visible"
flir_topic_name = "/lwir_image"
basler_topic_name = "/visible_image"
image_size = {'lwir': (320, 240), 'rgb': (320, 240)}

bridge = CvBridge()

def ros_spin_thread(): 
    rospy.spin()

class RosMultiespectralAcquire:
    def __init__(self, socketio):
        rospy.init_node('multiespectral_flask_gui') 

        self.running = False
        self.socketio = socketio
        
        self.client = []
        for ac_name in [flir_ac_name, basler_ac_name]:
            self.client.append(actionlib.SimpleActionClient(ac_name, multiespectral_fb.msg.MultiespectralAcquisitionAction))
            rospy.loginfo(f'Result: Wait for {ac_name} server')
            self.client[-1].wait_for_server()

        # Suscribirse a los topics de imagen
        self.image_sub1 = rospy.Subscriber(flir_topic_name, Image, self.lwir_image_cb)
        self.image_sub2 = rospy.Subscriber(basler_topic_name, Image, self.rgb_image_cb)

        self.ros_thread = threading.Thread(target=ros_spin_thread) 
        self.ros_thread.start()

        self.update_socketio_thread = threading.Thread(target=self.updateSocketio) 
        self.update_socketio_thread.start()
        self.socketio.on('image_size', self.update_image_size)
         
    def __del__(self):
        rospy.signal_shutdown("[MultiespectralAcquireGui]  Destructor") 
        self.ros_thread.join()
        self.update_socketio_thread.join()
    
    def cancelGoal(self):
        for client in self.client:
            client.cancel_goal()

    def stop(self):
        self.cancelGoal()
        frame_rate_lwir.stop()
        frame_rate_rgb.stop()
        rospy.loginfo("Finished image acquisition")

    def sendGoal(self, store):
        frame_rate_lwir.start()
        frame_rate_rgb.start()

        goal = multiespectral_fb.msg.MultiespectralAcquisitionGoal()
        goal.store = store

        for client in self.client:
            client.send_goal(goal, feedback_cb=self.feedback_cb)

        for client in self.client:
            client.wait_for_result()
            result = client.get_result()
            rospy.loginfo(f'Finished goal')

    def feedback_cb(self, feedback):
        global lwir_img_storepath, rgb_img_storepath
        # rospy.loginfo(f'Feedback: Images Acquired = {feedback.images_acquired}, Storage Path = {feedback.storage_path}')
        if 'lwir' in feedback.storage_path:
            lwir_img_storepath = feedback.storage_path
        else:
            rgb_img_storepath = feedback.storage_path

    def update_image_size(self, size_data):
        global image_size
        image_size['lwir'] = (size_data['lwir']['width'], size_data['lwir']['height'])
        image_size['rgb'] = (size_data['rgb']['width'], size_data['rgb']['height']) 

    def lwir_image_cb(self, msg):
        global lwir_img_path, total_images_received_lwir
        image = self.convert_image(msg)
        if image is not None:
            resized_image = cv2.resize(image, image_size['lwir'])
            filtered_image = cv2.bilateralFilter(resized_image, d=9, sigmaColor=75, sigmaSpace=75)
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            clahe_image = clahe.apply(filtered_image)
            _, lwir_buffer = cv2.imencode('.png', clahe_image)
            lwir_img_path = base64.b64encode(lwir_buffer).decode('utf-8')
            frame_rate_lwir.tick()
        else:
            rospy.logwarn("Failed to convert LWIR image.")

    def rgb_image_cb(self, msg):
        global rgb_img_path, total_images_received_rgb
        image = self.convert_image(msg)
        if image is not None:
            resized_image = cv2.resize(image, image_size['rgb'])
            _, rgb_buffer = cv2.imencode('.png', resized_image)
            rgb_img_path = base64.b64encode(rgb_buffer).decode('utf-8')
            frame_rate_rgb.tick()
        else:
            rospy.logwarn("Failed to convert RGB image.")

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
            time.sleep(0.5)
        
    def convert_image(self, ros_image):
        try:
            if ros_image.data:
                if ros_image.encoding == "mono8":
                    cv_image = bridge.imgmsg_to_cv2(ros_image, "mono8")
                elif ros_image.encoding == "bgr8":
                    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
                else:
                    rospy.logwarn(f"Unexpected image encoding: {ros_image.encoding}. Defaulting to 'bgr8'.")
                    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
                return cv_image
            else:
                rospy.logwarn("Received empty image message.")
                return None
        except CvBridgeError as e:
            rospy.logerr(f'CvBridge Error: {e}')
            return None