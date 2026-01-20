#include "camera_adapter_ros.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <filesystem>
#include "utils/logging_utils.h"
#include "utils/image_metadata.h"

CameraAdapterROS::CameraAdapterROS(const std::string& node_name)
    : rclcpp::Node(node_name),
      it_(std::shared_ptr<rclcpp::Node>(this, [](auto *) {})),
      cinfo_(nullptr)
{
    this->declare_parameter("dataset_output_path", "./");
    this->declare_parameter("image_topic", std::string(getType()+"_image"));
    this->declare_parameter("frame_rate", 5);
    this->declare_parameter("camera_ip", std::string(""));
    this->declare_parameter("camera_info_url", std::string(""));

    std::string dataset_output_path = this->get_parameter("dataset_output_path").as_string();
    this->topic_name = this->get_parameter("image_topic").as_string();
    this->frame_rate = this->get_parameter("frame_rate").as_int();
    this->camera_ip = this->get_parameter("camera_ip").as_string();
    this->camera_info_cfg = this->get_parameter("camera_info_url").as_string();

    this->img_path = dataset_output_path+std::string("/")+getFolderTimetag()+std::string("/")+getType()+std::string("/");
    std::filesystem::create_directories(img_path);
    logger_ = std::make_shared<RosLogger>(this->get_logger());
    logger_->info_stream() << "[CameraAdapterROS] Images will be stored in path: " << img_path;
    logger_->info_stream() << "[CameraAdapterROS] ROS parameters configured as: " << std::endl
        << " - image_topic: " << topic_name << std::endl
        << " - frame_rate: " << frame_rate;

    // Camera info manager initialization (only if config is provided)
    if (!this->camera_info_cfg.empty()) {
        logger_->info_stream() << "[CameraAdapterROS] Loading camera info from URL: " << this->camera_info_cfg;
        cinfo_ = std::make_shared<camera_info_manager::CameraInfoManager>(this->shared_from_this(), getName(), this->camera_info_cfg);
    } else {
        cinfo_ = nullptr;
        logger_->warn_stream() << "[CameraAdapterROS] No camera info URL provided for " << getName() << " camera.";
    }
    
    logger_ = std::make_shared<RosLogger>(this->get_logger());

    it_pub_ = it_.advertiseCamera(this->topic_name, 1);
}

bool CameraAdapterROS::publishImage(cv::Mat& curr_image, ImageMetadata& metadata)
{
    logger_->debug_stream() << "[CameraAdapterRos::publishImage] Init function.";
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
        logger_->debug_stream() << "[CameraAdapterRos::publishImage] Got image from "<<getName()<<", store with timestamp ("<<metadata.ros_timestamp<<") and publish it.";
        std_msgs::msg::Header header;
        uint64_t sec = metadata.ros_timestamp / 1000000000; 
        uint64_t nsec = metadata.ros_timestamp % 1000000000;
        header.stamp = metadata.ros_timestamp == 0 ? this->get_clock()->now() : rclcpp::Time(sec * 1e9 + nsec);
        header.frame_id = getName() + "_frame";
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, encoding, curr_image).toImageMsg();
        sensor_msgs::msg::CameraInfo cam_info;            
        if (cinfo_)
        {
            cam_info = cinfo_->getCameraInfo();
        }
        cam_info.header.stamp = msg->header.stamp; 
        it_pub_.publish(*msg.get(), cam_info);
        logger_->debug_stream() << "[CameraAdapterRos::publishImage] Published image from "<<getName()<<" with encoding: " << encoding;
    }
    if(curr_image.empty()) logger_->error_stream() << "[CameraAdapterRos::publishImage] Image is empty for " << getName() << " camera.";
    return true;
}

bool CameraAdapterROS::init_and_start_acquisition(int frame_rate) {
    this->frame_rate = frame_rate;
    bool result = CameraAdapter::init(frame_rate);

    if(!result) logger_->fatal_stream() << "[CameraAdapterROS::init] Could not configure " << getName() << " camera.";
    if(result) logger_->info_stream() << "[CameraAdapterROS::init] Initialized " << getName() << " camera.";
    
    result = result && beginAcquisition();
    
    if(result) logger_->info_stream() << SUCCEED_F << "[MADriver::init] Start image acquisition loop for camera "  << getName() << "." << RESET_F;
    if(!result)
    {
        logger_->fatal_stream() << "[CameraAdapterROS::init] Camera init image acquisition failed";
        throw std::runtime_error("[MAMaster::MAMaster] Camera init failed");
    }
    
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0/this->frame_rate),
        std::bind(&CameraAdapterROS::acquisition_loop, this));
    
        if (!result) {
        throw std::runtime_error("[CameraAdapterROS::init] Camera init failed");
    }
    
    logger_->info_stream() << SUCCEED_F << "[MADriver::MADriver] Camera "<<getName()<<" ("<<getType()<<") initialized successfully"<<RESET_F;
    return result;
}

