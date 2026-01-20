#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include "camera_drivers/camera_adapter.h"
#include "utils/logging_utils.h"


class RosLogger : public Logger {
    rclcpp::Logger ros_logger_;
public:
    RosLogger(rclcpp::Logger logger) : ros_logger_(logger) {}
    void debug(const std::string& msg) override { RCLCPP_DEBUG(ros_logger_, "%s", msg.c_str()); }
    void info(const std::string& msg) override { RCLCPP_INFO(ros_logger_, "%s", msg.c_str()); }
    void warn(const std::string& msg) override { RCLCPP_WARN(ros_logger_, "%s", msg.c_str()); }
    void error(const std::string& msg) override { RCLCPP_ERROR(ros_logger_, "%s", msg.c_str()); }
};


/**
 *   Merges both CameraAdapter with ROS2 Node functionalities.
 */
class CameraAdapterROS : public CameraAdapter, public rclcpp::Node {
protected:
    std::string topic_name;
    
    rclcpp::TimerBase::SharedPtr timer_;

    image_transport::ImageTransport it_;
    image_transport::CameraPublisher it_pub_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;
    
public:
    CameraAdapterROS(const std::string& node_name);
    bool publishImage(cv::Mat& curr_image, ImageMetadata& metadata);
    bool init_and_start_acquisition(int frame_rate);
};