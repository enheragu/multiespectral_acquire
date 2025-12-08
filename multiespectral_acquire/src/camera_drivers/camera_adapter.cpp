

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


void createTestPattern(cv::Mat& image) 
{
    // Chess pattern with red and green
    cv::rectangle(image, cv::Rect(0, 0, image.cols/2, image.rows/2), 
                  cv::Scalar(0, 0, 255), -1);  // Rojo
    cv::rectangle(image, cv::Rect(image.cols/2, 0, image.cols/2, image.rows/2), 
                  cv::Scalar(0, 255, 0), -1);   // Verde
    

    cv::putText(image, "TEST PATTERN", 
                cv::Point(image.cols/4, image.rows/2), 
                cv::FONT_HERSHEY_DUPLEX, 2.0, 
                cv::Scalar(255, 255, 255), 5);
    
    // Yellow border
    cv::rectangle(image, cv::Point(10, 10), 
                  cv::Point(image.cols-10, image.rows-10), 
                  cv::Scalar(0, 255, 255), 8);  // Amarillo grueso
}

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
    bool result = initCamera(frame_rate, this->camera_ip);
    if(!result) RCLCPP_FATAL_STREAM(get_logger(),"[MAT::init] Could not initialize " << getName() << " camera.");
    if (!this->camera_info_cfg.empty())
    {
        RCLCPP_INFO_STREAM(get_logger(),"[MAT::init] Loading camera info from URL: " << this->camera_info_cfg);
        cinfo_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, getName(), this->camera_info_cfg);
    }
    else
    {
        cinfo_ = nullptr;
        RCLCPP_WARN_STREAM(get_logger(),"[MAT::init] No camera info URL provided for " << getName() << " camera.");
    }

    
    return result;
}

bool MultiespectralAcquireT::changeFrameRate(int frame_rate)
{
    RCLCPP_INFO_STREAM(get_logger(),"[MAT::changeFrameRate] Closing camera to setup new frame rate to: " << frame_rate);
    bool result = closeCamera();
    this->init(frame_rate);
    return result;
}


MultiespectralAcquireT::MultiespectralAcquireT(std::string name): Node(name),
            node_handle_(std::shared_ptr<MultiespectralAcquireT>(this, [](auto *) {})),
            it_(node_handle_)
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

    this->img_path = dataset_output_path+std::string("/")+getFolderTimetag()+std::string("/")+getType()+std::string("/");
    std::filesystem::create_directories(img_path);
    RCLCPP_INFO_STREAM(get_logger(),"[MAT::MAT] Images will be stored in path: " << img_path);
    RCLCPP_INFO_STREAM(get_logger(),"[MAT::MAT] ROS parameters configured as: " << std::endl
        << " - image_topic: " << topic_name << std::endl
        << " - frame_rate: " << frame_rate);

    this->camera_info_cfg = this->get_parameter("camera_info_url").as_string();
    
    image_pub_ = it_.advertiseCamera(this->topic_name, 1);   
}

MultiespectralAcquireT::~MultiespectralAcquireT(void)
{   
    const std::scoped_lock<std::mutex> lock(camera_mutex);
    RCLCPP_INFO_STREAM(get_logger(),"[MAT::~MAT] Closing camera " << getName() << " on destructor.");
    bool result = closeCamera();
    if(!result) RCLCPP_FATAL_STREAM(get_logger(),"[MAT::~MAT] Could not finish correctly " << getName() << " camera.");
    if(result) RCLCPP_INFO_STREAM(get_logger(),"[MAT::~MAT] Correctly finished " << getName() << " camera.");
}

bool MultiespectralAcquireT::grabImage(cv::Mat& curr_image, uint64_t& timestamp, ImageMetadata& metadata)
{
    // RCLCPP_DEBUG_STREAM(get_logger(),"[MAT::grabImage] Init function.");
    const std::scoped_lock<std::mutex> lock(camera_mutex);
    bool result =  acquireImage(curr_image, timestamp, metadata);

    // TBD use camera timestamp once synchronized
    rclcpp::Time now = this->get_clock()->now();
    timestamp = now.nanoseconds();

    // RCLCPP_DEBUG_STREAM(get_logger(),"[MAT::grabImage] End function.");
    if(!result) RCLCPP_ERROR_STREAM(get_logger(),"[MAT::grabImage] Could not acquire image from " << getName() << " camera.");
    return result;
}

bool MultiespectralAcquireT::processImage(cv::Mat& curr_image, uint64_t& timestamp, ImageMetadata& metadata, bool store)
{
    RCLCPP_DEBUG_STREAM(get_logger(),"[MAT::processImage] Init function.");
    const std::scoped_lock<std::mutex> lock(camera_mutex);
    if (!curr_image.empty() && store) 
    {
        std::ostringstream filename;
        filename << img_path << "/" << getTimeTag() << "_tcam_" << timestamp;
        cv::imwrite(filename.str().c_str()+std::string(".png"), curr_image);

        saveMetadataYaml(metadata, filename.str().c_str()+std::string(".yaml"));
    } 
    // RCLCPP_DEBUG_STREAM(get_logger(),"[MAT::processImage] Finish storing image.");
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
        // RCLCPP_DEBUG_STREAM(get_logger(),"[MAT::processImage] Got image from "<<getName()<<", store with timestamp ("<<timestamp<<") and publish it.");
        std_msgs::msg::Header header;
        // Convertir timestamp_ns a segundos y nanosegundos 
        uint64_t sec = timestamp / 1000000000; 
        uint64_t nsec = timestamp % 1000000000;
        header.stamp = timestamp == 0 ? this->get_clock()->now() : rclcpp::Time(sec * 1e9 + nsec);
        header.frame_id = getName() + "_frame";
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, encoding, curr_image).toImageMsg();
        
        sensor_msgs::msg::CameraInfo cam_info;            
        if (cinfo_)
        {
            cam_info = cinfo_->getCameraInfo();
        }
        cam_info.header.stamp = msg->header.stamp; 
        image_pub_.publish(*msg.get(), cam_info);
    
        // ros::spinOnce(); // without explicit spinOnce, the LWIR image is as black (rgb is ok...). The info stream also works?¿
        // RCLCPP_DEBUG_STREAM(get_logger(),"[MAT::processImage] Published image from "<<getName()<<" with encoding: " << encoding);
    }

    if(curr_image.empty()) RCLCPP_ERROR_STREAM(get_logger(), "[MAT::processImage] Image is empty for " << getName() << " camera.");
    return true;
}

bool MultiespectralAcquireT::grabStoreImage(cv::Mat& curr_image, uint64_t& timestamp, ImageMetadata& metadata, bool store)
{
    // RCLCPP_DEBUG(get_logger(), "[MAT::grabStoreImage] Grabbing image.");
    bool result =  grabImage(curr_image, timestamp, metadata);
    // RCLCPP_DEBUG(get_logger(), "[MAT::grabStoreImage] Storing image.");
    result = result && processImage(curr_image, timestamp, metadata, store);
    // RCLCPP_DEBUG(get_logger(), "[MAT::grabStoreImage] Image processed.");
    return result;
}

