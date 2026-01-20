
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include "camera_drivers/camera_adapter.h"
#include "utils/logging_utils.h"


class RosLogger : public Logger {
public:
    RosLogger() {}
    void debug(const std::string& msg) override { ROS_DEBUG("%s", msg.c_str()); }
    void info(const std::string& msg) override { ROS_INFO("%s", msg.c_str()); }
    void warn(const std::string& msg) override { ROS_WARN("%s", msg.c_str()); }
    void error(const std::string& msg) override { ROS_ERROR("%s", msg.c_str()); }
};


/**
 *   Merges both CameraAdapter with ROS2 Node functionalities.
 */
class CameraAdapterROS : public CameraAdapter {
    ros::NodeHandle nh_;
protected:
    std::string topic_name;
    
    ros::Timer::SharedPtr timer_;

    image_transport::ImageTransport it_;
    image_transport::CameraPublisher it_pub_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;
    
public:
    CameraAdapterROS(const std::string& node_name);
    bool publishImage(cv::Mat& curr_image, ImageMetadata& metadata);
    bool init_and_start_acquisition(int frame_rate);
};