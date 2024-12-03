import rospy
import actionlib
from multiespectral_fb.MultiespectralAcquisitionAction import MultiespectralAcquisitionAction, MultiespectralAcquisitionGoal
from sensor_msgs.msg import Image
import cv2
import base64
import os

from FreqCounter import FreqCounter

frame_rate_lwir = FreqCounter()
frame_rate_rgb = FreqCounter()
lwir_img_path = ""
rgb_img_path = ""
lwir_img_storepath = ""
rgb_img_storepath = ""
store_in_drive = False
camera_handler = None

flir_ac_name = None
basler_ac_name = ""
class RosMultiespectralAcquire:
    def __init__(self, socketio):
        self.running = False
        self.socketio = socketio

        self.client = []
        for ac_name in [flir_ac_name, basler_ac_name]:
            self.client.append(actionlib.SimpleActionClient(ac_name, MultiespectralAcquisitionAction))
            self.client[-1].wait_for_server()

        # Suscribirse a los topics de imagen
        self.image_sub1 = rospy.Subscriber('/lwir_image', Image, self.lwir_image_cb)
        self.image_sub2 = rospy.Subscriber('/visible_image', Image, self.rgb_image_cb)
        
    def cancel_goal(self):
        for client in self.client:
            client.cancel_goal()


    def send_goal(self, store):
        frame_rate_lwir.start()
        frame_rate_rgb.start()

        goal = MultiespectralAcquisitionGoal(store=store)

        for client in self.client:
            client.send_goal(goal, feedback_cb=self.feedback_cb)
        for client in self.client:
            client.wait_for_result()
            result = self.client.get_result()
            rospy.loginfo(f'Result: Images Acquired = {result.images_acquired}')

    def feedback_cb(self, feedback):
        global lwir_img_storepath, rgb_img_storepath
        # rospy.loginfo(f'Feedback: Images Acquired = {feedback.images_acquired}, Storage Path = {feedback.storage_path}')
        if 'lwir' in feedback.storage_path:
            lwir_img_storepath = feedback.storage_path
        else:
            rgb_img_storepath = feedback.storage_path
        self.updateSocketio()

    def lwir_image_cb(self, msg):
        global lwir_img_path, total_images_received_lwir
        # rospy.loginfo("Image received from lwir camera")
        image = self.convert_image(msg)
        _, lwir_buffer = cv2.imencode('.jpg', image)
        lwir_img_path = base64.b64encode(lwir_buffer).decode('utf-8')
        frame_rate_lwir.tick()
        self.updateSocketio()
            
    def rgb_image_cb(self, msg):
        global rgb_img_path, total_images_received_rgb
        # rospy.loginfo("Image received from rgb camera")
        image = self.convert_image(msg)
        _, rgb_buffer = cv2.imencode('.jpg', image)
        rgb_img_path = base64.b64encode(rgb_buffer).decode('utf-8')
        frame_rate_rgb.tick()
        self.updateSocketio()

    def updateSocketio(self):
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
        
    def convert_image(self, ros_image):
        # Convertir el mensaje de imagen de ROS a una imagen de OpenCV
        try:
            cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
            return cv_image
        except CvBridgeError as e:
            rospy.logerr(f'CvBridge Error: {e}')
            return None

if __name__ == '__main__':
    rospy.init_node('image_acquire_client')
    client = RosMultiespectralAcquire()
    
    # Enviar una meta para almacenar imágenes
    client.send_goal(store=True)

    rospy.spin()
