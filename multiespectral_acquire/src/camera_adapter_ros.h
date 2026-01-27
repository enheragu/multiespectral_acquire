
#ifndef CAMERA_ADAPTER_ROS_H
#define CAMERA_ADAPTER_ROS_H

#include <vector>
#include <algorithm> 

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>

#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>

#include "camera_drivers/camera_adapter.h"
#include "utils/logging_utils.h"
#include "utils/timed_frame_buffer.h"


class RosLogger : public Logger {
public:
    RosLogger() {}
    void debug(const std::string& msg) override { ROS_DEBUG("%s", msg.c_str()); }
    void info(const std::string& msg) override { ROS_INFO("%s", msg.c_str()); }
    void warn(const std::string& msg) override { ROS_WARN("%s", msg.c_str()); }
    void error(const std::string& msg) override { ROS_ERROR("%s", msg.c_str()); }
    void fatal(const std::string& msg) override { ROS_FATAL("%s", msg.c_str()); }
};

struct GNSSDataBuffer {
    uint64_t timestamp;
    GNSSMetadata gnss_data;
};    
struct OdomDataBuffer {
    uint64_t timestamp;
    OdomMetadata odom_data;
};

/**
 *   Merges both CameraAdapter with ROS1 Node functionalities.
 */
class CameraAdapterROS : public CameraAdapter {
protected:
    ros::NodeHandle nh_;
    std::string topic_name;
    
    ros::Timer timer_;

    image_transport::ImageTransport it_;
    image_transport::Publisher it_pub_;
    ros::Publisher cam_info_pub_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;

    ros::Subscriber gnss_data_sub_;
    ros::Subscriber odom_data_sub_;

    TimedFrameBuffer<GNSSDataBuffer> gnss_data_buffer_;
    TimedFrameBuffer<OdomDataBuffer> odom_data_buffer_;
    
public:
    CameraAdapterROS(const std::string& node_name);
    bool publishImage(cv::Mat& curr_image, ImageMetadata& metadata);
    bool init_and_start_acquisition(int frame_rate);
    virtual void acquisition_loop(const ros::TimerEvent&) = 0;

    void gnss_data_cb(const sensor_msgs::NavSatFix::ConstPtr& msg)
    {
        std::vector<float> cov(9);
        std::copy(msg->position_covariance.data(), msg->position_covariance.data() + 9, cov.begin());
        gnss_data_buffer_.addFrame(GNSSDataBuffer{msg->header.stamp.toNSec(), GNSSMetadata(msg->latitude, msg->longitude, msg->altitude, cov)});
    }

    void odom_data_cb(const nav_msgs::Odometry::ConstPtr& msg) {
        std::vector<float> cov(36);
        std::copy(msg->pose.covariance.data(), msg->pose.covariance.data() + 36, cov.begin());
        odom_data_buffer_.addFrame(OdomDataBuffer{msg->header.stamp.toNSec(), 
            OdomMetadata(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z,
                         msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w,
                         cov)
        }); 
    }
};

#endif // CAMERA_ADAPTER_ROS_H