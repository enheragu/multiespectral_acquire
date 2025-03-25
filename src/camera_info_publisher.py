#!/usr/bin/env python3
import rospy
import yaml
from sensor_msgs.msg import CameraInfo

def load_camera_info(yaml_path):
    
    with open(yaml_path, 'r') as file:
        return yaml.safe_load(file)

def get_camera_info(camera_info_yaml):
    camera_info_msg = CameraInfo()
    camera_info_msg.header.stamp = rospy.Time.now()
    camera_info_msg.header.frame_id = camera_info_yaml['camera_name']
    camera_info_msg.width = camera_info_yaml['image_width']
    camera_info_msg.height = camera_info_yaml['image_height']
    camera_info_msg.distortion_model = camera_info_yaml['distortion_model']
    camera_info_msg.D = camera_info_yaml['distortion_coefficients']['data']
    camera_info_msg.K = camera_info_yaml['camera_matrix']['data']
    camera_info_msg.R = camera_info_yaml.get('rectification_matrix', {'data': [1, 0, 0, 0, 1, 0, 0, 0, 1]})['data']
    camera_info_msg.P = camera_info_yaml['projection_matrix']['data']
    
    return camera_info_msg

def camera_info_publisher():
    rospy.init_node('camera_info_publisher', anonymous=True)

    image_topic = rospy.get_param("~image_topic", 'default/image_topic') + '/camera_info'
    yaml_calibration = rospy.get_param("~calibration", '/default/path/to/calibration.yaml')
    frame_rate = rospy.get_param("~frame_rate", 5)

    try:
        camera_info_yaml = load_camera_info(yaml_calibration)
    except FileNotFoundError:
        rospy.logerr(f"Could not find calibration information for the camera: {yaml_calibration}")
        return
    except yaml.YAMLError as e:
        rospy.logerr(f"Error parsing YAML file: {e}")
        return

    pub = rospy.Publisher(image_topic, CameraInfo, queue_size=10)
    rate = rospy.Rate(frame_rate)

    rospy.loginfo(f"Publish ({frame_rate} Hz) CameraInfo to: {image_topic}")
    while not rospy.is_shutdown():
        pub.publish(get_camera_info(camera_info_yaml))
        rate.sleep()

if __name__ == '__main__':
    try:
        camera_info_publisher()
    except rospy.ROSInterruptException:
        pass
