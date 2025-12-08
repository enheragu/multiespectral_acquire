

#include <mutex>
#include <iostream>
#include <sstream>
#include <fstream>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"

#include "camera_adapter.h"


std::string getFolderTimetag()
{
    auto now = std::chrono::system_clock::now();
    std::time_t timeNow = std::chrono::system_clock::to_time_t(now);
    
    std::tm* tmNow = std::localtime(&timeNow);

    std::ostringstream oss;
    oss << std::put_time(tmNow, "%y-%m-%d_%H-%M");

    return oss.str();
}

std::string getTimeTag() {
    auto now = std::chrono::system_clock::now();
    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    std::time_t timeNow = std::chrono::system_clock::to_time_t(now);
    
    std::tm* tmNow = std::localtime(&timeNow);

    std::ostringstream oss;
    oss << std::put_time(tmNow, "%y-%m-%d_%H-%M-%S_") << std::setw(3) << std::setfill('0') << millis.count();

    return oss.str();
}

void saveMetadataYaml(const ImageMetadata& meta, const std::string& filename)
{
    YAML::Node node;
    node["systemTime"] = meta.systemTime;
    node["timestamp"] = meta.timestamp;
    node["frameCounter"] = meta.frameCounter;
    node["exposureTime_us"] = meta.exposureTime;
    node["gain_dB"] = meta.gain;
    node["width"] = meta.width;
    node["height"] = meta.height;
    node["pixelFormat"] = meta.pixelFormat;

    std::ofstream fout(filename);
    fout << node;
}


bool MultiespectralAcquireT::init(int frame_rate)
{    

    rclcpp::Node::SharedPtr node_shared = rclcpp::Node::SharedPtr(this);
    std::string camera_name = getName();
    std::string url = this->get_parameter("camera_info_url").as_string();
    cinfo_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, camera_name, url);

    std::shared_ptr<image_transport::ImageTransport> it = std::make_shared<image_transport::ImageTransport>(node_shared);
    RCLCPP_INFO_STREAM(get_logger(),"[MAT] Images for  " << getName() << " camera will be published to topic: " << topic_name);
    image_pub_ = it->advertiseCamera(this->topic_name, 1);

    bool result = initCamera(frame_rate, this->camera_ip);
    if(!result) RCLCPP_FATAL_STREAM(get_logger(),"[MAT] Could not initialize " << getName() << " camera.");
    return result;
}

bool MultiespectralAcquireT::changeFrameRate(int frame_rate)
{
    RCLCPP_INFO_STREAM(get_logger(),"[MAT::changeFrameRate] Closing camera to setup new frame rate to: " << frame_rate);
    bool result = closeCamera();
    this->init(frame_rate);
    return result;
}


MultiespectralAcquireT::MultiespectralAcquireT(std::string name): Node(name)
{
    this->declare_parameter("dataset_output_path", "./");
    this->declare_parameter("image_topic", std::string(getType()+"_image"));
    this->declare_parameter("frame_rate", 10);
    this->declare_parameter("camera_ip", std::string(""));


    std::string dataset_output_path = this->get_parameter("dataset_output_path").as_string();
    this->topic_name = this->get_parameter("image_topic").as_string();
    int frame_rate = this->get_parameter("frame_rate").as_int();
    this->camera_ip = this->get_parameter("camera_ip").as_string();

    this->img_path = dataset_output_path+std::string("/")+getFolderTimetag()+std::string("/")+getType()+std::string("/");
    std::filesystem::create_directories(img_path);
    RCLCPP_INFO_STREAM(get_logger(),"[MAT::MultiespectralAcquireT] Images will be stored in path: " << img_path);
    

    this->declare_parameter("camera_info_url", 
        "package://multiespectral_acquisition/conf/camera_params.yaml");
    this->camera_info_cfg = dataset_output_path+std::string("/")+getType()+std::string("_camera_info.yaml");

    bool result = this->init(frame_rate);
    
    if (!result) 
    {
        RCLCPP_FATAL_STREAM(get_logger(),"[MAT::MultiespectralAcquireT] Camera init failed");
        throw std::runtime_error("[MAT::MultiespectralAcquireT] Camera init failed");
    }
}

MultiespectralAcquireT::~MultiespectralAcquireT(void)
{   
    const std::scoped_lock<std::mutex> lock(camera_mutex);
    bool result = closeCamera();
    if(!result) RCLCPP_FATAL_STREAM(get_logger(),"[MAT] Could not finish correctly " << getName() << " camera.");
    if(result) RCLCPP_INFO_STREAM(get_logger(),"[MAT] Correctly finished " << getName() << " camera.");
}

bool MultiespectralAcquireT::grabImage(cv::Mat& curr_image, uint64_t& timestamp, ImageMetadata& metadata)
{
    const std::scoped_lock<std::mutex> lock(camera_mutex);
    bool result =  acquireImage(curr_image, timestamp, metadata);

    // TBD use camera timestamp once synchronized
    rclcpp::Time now = this->get_clock()->now();
    timestamp = now.nanoseconds();

    if(!result) RCLCPP_ERROR_STREAM(get_logger(),"[MAT::grabImage] Could not acquire image from " << getName() << " camera.");
    return result;
}

bool MultiespectralAcquireT::StoreImage(cv::Mat& curr_image, uint64_t& timestamp, ImageMetadata& metadata, bool store)
{
    const std::scoped_lock<std::mutex> lock(camera_mutex);
    if (!curr_image.empty() && store) 
    {
        std::ostringstream filename;
        filename << img_path << "/" << getTimeTag() << "_tcam_" << timestamp;
        cv::imwrite(filename.str().c_str()+std::string(".png"), curr_image);

        saveMetadataYaml(metadata, filename.str().c_str()+std::string(".yaml"));
    } 
    
    // Convert to a sensor_msgs::msg::Image message detecting encoding
    if (!curr_image.empty())
    {
        std::string encoding;
        if (curr_image.type() == CV_8UC3) {
            encoding = "bgr8";
        } else if (curr_image.type() == CV_8UC1) {
            encoding = "mono8";
        } else {
            std::cerr << "Unsupported image type: " << curr_image.type() << std::endl;
            return false;
        }
        // RCLCPP_INFO_STREAM(get_logger(),"[MAT::StoreImage] Got image from "<<getName()<<", store with timestamp ("<<timestamp<<") and publish it.");
        std_msgs::msg::Header header;
        // Convertir timestamp_ns a segundos y nanosegundos 
        uint64_t sec = timestamp / 1000000000; 
        uint64_t nsec = timestamp % 1000000000;
        header.stamp = timestamp == 0 ? this->get_clock()->now() : rclcpp::Time(sec * 1e9 + nsec);
        header.frame_id = getName() + "_frame";
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, encoding, curr_image).toImageMsg();
        
        sensor_msgs::msg::CameraInfo cam_info = cinfo_->getCameraInfo();
        cam_info.header.stamp = msg->header.stamp; 
        image_pub_.publish(*msg.get(), cam_info);
    
        // ros::spinOnce(); // without explicit spinOnce, the LWIR image is as black (rgb is ok...). The info stream also works?¿
        // RCLCPP_INFO_STREAM(get_logger(),"[MAT::StoreImage] Published image from "<<getName()<<" with encoding: " << encoding);
    }

    if(curr_image.empty()) RCLCPP_ERROR_STREAM(get_logger(), "[MAT::StoreImage] Image is empty for " << getName() << " camera.");
    return true;
}

bool MultiespectralAcquireT::grabStoreImage(cv::Mat& curr_image, uint64_t& timestamp, ImageMetadata& metadata, bool store)
{
    // RCLCPP_INFO("[MAT::grabStoreImage] Grabbing image.");
    bool result =  grabImage(curr_image, timestamp, metadata);
    // RCLCPP_INFO("[MAT::grabStoreImage] Storing image.");
    result = result && StoreImage(curr_image, timestamp, metadata, store);
    // RCLCPP_INFO("[MAT::grabStoreImage] Image stored.");
    return result;
}

