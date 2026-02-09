/**
 * @file    camera_handler_node.cpp
 * @brief   Simplified camera node that only acquires and publishes images with metadata
 */

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>

#include "multiespectral_acquire/ImageWithMetadata.h"
#include "core/camera_drivers/camera_adapter.h"
#include "core/utils/logging_utils.h"
#include "core/utils/image_metadata.h"
#include "ros_utils/ros_logger.h"

void fillImageMetadataMsg(const ImageMetadata& meta, multiespectral_acquire::ImageMetadata& msg) {
    msg.camera_timestamp = meta.camera_timestamp;
    msg.exposure_time = meta.exposureTime;
    msg.half_exposure_timestamp = meta.half_exposure_timestamp;
    msg.frame_counter = meta.frameCounter;
    msg.gain = meta.gain;
    msg.width = meta.width;
    msg.height = meta.height;
    msg.pixel_format = meta.pixelFormat;
    msg.timetag = meta.timetag;
    msg.img_name = meta.img_name;
    msg.img_pair_name = meta.img_pair_name;
    msg.dataset_name = meta.dataset_name;
}

class CameraHandlerNode : public CameraAdapter {
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher pub_image_metadata_;
    image_transport::ImageTransport it_;
    image_transport::Publisher it_pub_;
    ros::Publisher cam_info_pub_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;
    
    ros::Timer timer_;
    std::string topic_name_;
    std::string camera_info_cfg_;
    
    // Error tracking for graceful degradation
    int consecutive_errors_ = 0;
    static constexpr int MAX_CONSECUTIVE_ERRORS = 10;
    
public:
    CameraHandlerNode() : nh_(), pnh_("~"), it_(nh_) {
        this->setLogger(std::make_shared<RosLogger>());
        
        pnh_.param<std::string>("image_topic", topic_name_, getType() + "_image");
        pnh_.param<double>("output_frame_rate", frame_rate, 5.0);
        pnh_.param<std::string>("camera_ip", camera_ip, "");
        pnh_.param<std::string>("camera_info_url", camera_info_cfg_, "");
        
        // PTP configuration (set before init)
        bool use_ptp;
        pnh_.param<bool>("use_ptp", use_ptp, true);  // Default: try to use PTP
        force_disable_ptp = !use_ptp;  // Invert logic: use_ptp=false means force_disable=true
        
        // Session/dataset name for metadata
        std::string dataset_name_param;
        pnh_.param<std::string>("dataset_name", dataset_name_param, getFolderTimetag());
        dataset_name = dataset_name_param;
        
        // Set node name for better logging
        setNodeName(ros::this_node::getName());
        
        logger_->info_stream() << "[CameraHandlerNode] Configured: topic=" << topic_name_ 
                               << ", rate=" << frame_rate << "Hz, session=" << dataset_name;
        
        // Publishers
        pub_image_metadata_ = nh_.advertise<multiespectral_acquire::ImageWithMetadata>(topic_name_ + "/image_with_metadata", 1);
        it_pub_ = it_.advertise(topic_name_, 1);
        cam_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(topic_name_ + "/camera_info", 1);
        
        // Initialize camera
        if (!this->init(frame_rate)) {
            logger_->fatal_stream() << "[CameraHandlerNode] Failed to initialize camera";
            ros::shutdown();
            return;
        }
        
        // Camera info manager
        if (!camera_info_cfg_.empty()) {
            cinfo_ = std::make_shared<camera_info_manager::CameraInfoManager>(nh_, getModelName(), camera_info_cfg_);
        }
        
        // Start acquisition timer
        timer_ = nh_.createTimer(ros::Duration(1.0/frame_rate), &CameraHandlerNode::acquisition_cb, this);
        
        logger_->info_stream() << "[CameraHandlerNode] " << getName() << " initialized successfully at "
                               << "IP: " << camera_ip << ", Frame Rate: " << frame_rate << " Hz";
    }
    
    void acquisition_cb(const ros::TimerEvent&) {
        cv::Mat curr_image;
        
        // default test image for debugging
        curr_image = cv::Mat(480, 640, CV_8UC1, cv::Scalar(0));
        createTestPattern(curr_image);

        ImageMetadata metadata;
        metadata.dataset_name = dataset_name;
        metadata.setROSTimeNowCallback([]() { return static_cast<uint64_t>(ros::Time::now().toNSec()); });
        
        if (!grabImage(curr_image, metadata) || curr_image.empty()) {
            logger_->error_stream() << "[CameraHandlerNode] Failed to grab image";
            return;
        }

        std::string encoding;
        if (curr_image.type() == CV_8UC3) {
            encoding = "bgr8";
        } else if (curr_image.type() == CV_8UC1) {
            encoding = "mono8";
        } else {
            logger_->error_stream() << "Unsupported image type: " << curr_image.type() << std::endl;
            return;
        }
        
        // Publish image (for visualization)
        std_msgs::Header header;
        // Use camera_timestamp in header (ROS convention)
        header.stamp.sec = metadata.camera_timestamp / 1000000000ULL;
        header.stamp.nsec = metadata.camera_timestamp % 1000000000ULL;
        header.frame_id = getName() + "_frame";
        
        sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(header, encoding, curr_image).toImageMsg();
        it_pub_.publish(img_msg);
        
        // Publish camera info
        if (cinfo_) {
            sensor_msgs::CameraInfo cam_info = cinfo_->getCameraInfo();
            cam_info.header = header;
            cam_info_pub_.publish(cam_info);
        }
        
        // Publish image with metadata (for storage nodes)
        multiespectral_acquire::ImageWithMetadata msg;
        msg.image = *img_msg;
        fillImageMetadataMsg(metadata, msg.metadata);
        msg.metadata.header = header;
        pub_image_metadata_.publish(msg);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "camera_handler_node");
    ROS_INFO_STREAM("[camera_handler_node] Starting Camera Handler Node for " << getType() << " images");
    CameraHandlerNode node;
    ros::spin();
    return 0;
}
