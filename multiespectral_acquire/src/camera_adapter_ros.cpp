#include "camera_adapter_ros.h"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <filesystem>
#include "utils/logging_utils.h"
#include "utils/image_metadata.h"

CameraAdapterROS::CameraAdapterROS(const std::string& node_name)
        : nh_(),
          it_(nh_),
          cinfo_(nullptr)
{
    this->setLogger(std::make_shared<RosLogger>());

    // Use a private NodeHandle to read node-private parameters (do not call nh_.param with '~' keys)
    ros::NodeHandle pnh("~");
    std::string dataset_output_path;
    pnh.param<std::string>("dataset_output_path", dataset_output_path, "./");
    pnh.param<std::string>("image_topic", topic_name, getType()+"_image");
    pnh.param<int>("frame_rate", frame_rate, 5);
    pnh.param<std::string>("camera_ip", camera_ip, "");
    pnh.param<std::string>("camera_info_url", camera_info_cfg, "");

    this->init_store_folder(dataset_output_path);
    logger_->info_stream() << "[CameraAdapterROS] ROS parameters configured as: " << std::endl
        << " - image_topic: " << topic_name << std::endl
        << " - frame_rate: " << frame_rate;

    // Advertise image with image_transport and publish CameraInfo separately so we can
    // optionally add a tag/prefix only to the camera_info topic.
    it_pub_ = it_.advertise(topic_name, 1);
    std::string cam_info_topic = topic_name + std::string("/camera_info");
    cam_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(cam_info_topic, 1);

    std::string gnss_topic_name, odom_topic_name;
    pnh.param<std::string>("gnss_topic", gnss_topic_name, "-");
    pnh.param<std::string>("odom_topic", odom_topic_name, "-");
    if (gnss_topic_name != "-")
    {
        logger_->info_stream() << "[CameraAdapterROS] Subscribing to GNSS topic: " << gnss_topic_name;
        gnss_data_sub_ = nh_.subscribe(gnss_topic_name, 10, &CameraAdapterROS::gnss_data_cb, this);
    }
    if (odom_topic_name != "-") 
    {
        logger_->info_stream() << "[CameraAdapterROS] Subscribing to Odom topic: " << odom_topic_name;
        odom_data_sub_ = nh_.subscribe(odom_topic_name, 10, &CameraAdapterROS::odom_data_cb, this);
    }
}

bool CameraAdapterROS::publishImage(cv::Mat& curr_image, ImageMetadata& metadata)
{
    logger_->debug_stream() << "[CameraAdapterROS::publishImage] Init function.";
    if (!curr_image.empty())
    {
        const std::scoped_lock<std::mutex> lock(camera_mutex);
        std::string encoding;
        if (curr_image.type() == CV_8UC3) {
            encoding = "bgr8";
        } else if (curr_image.type() == CV_8UC1) {
            encoding = "mono8";
        } else {
            std::cerr << "Unsupported image type: " << curr_image.type() << std::endl;
            return false;
        }
        logger_->debug_stream() << "[CameraAdapterROS::publishImage] Got image from "<<getName()<<", store with timestamp ("<<metadata.ros_timestamp<<") and publish it.";
        std_msgs::Header header;
        uint64_t sec = metadata.ros_timestamp / 1000000000; 
        uint64_t nsec = metadata.ros_timestamp % 1000000000;
        header.stamp.sec = sec;
        header.stamp.nsec = nsec;
        header.frame_id = getName() + "_frame";
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, encoding, curr_image).toImageMsg();
        sensor_msgs::CameraInfo cam_info_msg;            
        if (cinfo_)
        {
            cam_info_msg = cinfo_->getCameraInfo();
        }
        cam_info_msg.header.stamp = msg->header.stamp; 
        sensor_msgs::CameraInfoPtr cam_info(new sensor_msgs::CameraInfo(cam_info_msg));
        it_pub_.publish(msg);
        // Publish CameraInfo on the separate topic (possibly prefixed)
        cam_info_pub_.publish(*cam_info);
        logger_->debug_stream() << "[CameraAdapterROS::publishImage] Published image from "<<getName()<<" with encoding: " << encoding;
    }
    if(curr_image.empty()) logger_->error_stream() << "[CameraAdapterROS::publishImage] Image is empty for " << getName() << " camera.";
    return true;
}

bool CameraAdapterROS::init_and_start_acquisition(int frame_rate) {
    this->frame_rate = frame_rate;
    bool result = CameraAdapter::init(frame_rate);
    
    // Camera info manager initialization (only if config is provided) -> needs camera data, so it needs to be configured :)
    if (!camera_info_cfg.empty()) {
        logger_->info_stream() << "[CameraAdapterROS] Loading camera info from URL: " << camera_info_cfg;
        cinfo_ = std::make_shared<camera_info_manager::CameraInfoManager>(nh_, getName(), camera_info_cfg);
    } else {
        cinfo_ = nullptr;
        logger_->warn_stream() << "[CameraAdapterROS] No camera info URL provided for " << getName() << " camera.";
    }

    timer_ = nh_.createTimer(ros::Duration(1.0/this->frame_rate), &CameraAdapterROS::acquisition_loop, this);
    
    logger_->info_stream() << SUCCEED_F << "[MADriver::MADriver] Camera "<<getName()<<" ("<<getType()<<") initialized successfully"<<RESET_F;
    return result;
}

