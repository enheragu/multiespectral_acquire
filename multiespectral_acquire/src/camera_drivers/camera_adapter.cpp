#include <mutex>
#include <iostream>
#include <sstream>
#include <fstream>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "utils/image_metadata.h"

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


bool CameraAdapter::init(int frame_rate)
{    
    bool result = initCamera(frame_rate, this->camera_ip);
    if(!result) logger_->error_stream() << "[CameraAdapter::init] Could not initialize " << getName() << " camera.";

    return result;
}

bool CameraAdapter::changeFrameRate(int frame_rate)
{
    logger_->info_stream() << "[CameraAdapter::changeFrameRate] Closing camera to setup new frame rate to: " << frame_rate;
    bool result = closeCamera();
    this->init(frame_rate);
    return result;
}


CameraAdapter::~CameraAdapter(void)
{   
    const std::scoped_lock<std::mutex> lock(camera_mutex);
    logger_->info_stream() << "[CameraAdapter::~CameraAdapter] Closing camera " << getName() << " on destructor.";
    bool result = closeCamera();
    if(!result) logger_->error_stream() << "[CameraAdapter::~CameraAdapter] Could not finish correctly " << getName() << " camera.";
    if(result) logger_->info_stream() << "[CameraAdapter::~CameraAdapter] Correctly finished " << getName() << " camera.";
}

bool CameraAdapter::grabImage(cv::Mat& curr_image, ImageMetadata& metadata)
{
    logger_->debug_stream() << "[CameraAdapter::grabImage] Command to acquire image.";
    const std::scoped_lock<std::mutex> lock(camera_mutex);
    bool result =  acquireImage(curr_image, metadata);

    logger_->debug_stream() << "[CameraAdapter::grabImage] Aquired image.";
    if(!result) logger_->error_stream() << "[CameraAdapter::grabImage] Could not acquire image from " << getName() << " camera.";
    return result;
}

bool CameraAdapter::storeImage(cv::Mat& curr_image, ImageMetadata& metadata)
{
    
    if (!curr_image.empty()) 
    {
        const std::scoped_lock<std::mutex> lock(camera_mutex);
        std::ostringstream oss;
        oss << std::setfill('0') << std::setw(6) << this->stored_images;
        metadata.img_name = getType() + "_" + oss.str();
        this->stored_images++;

        std::ostringstream filename;
        filename << this->img_path << "/" << metadata.img_name;
        cv::imwrite(filename.str().c_str()+std::string(".png"), curr_image);

        saveMetadataYaml(metadata, filename.str().c_str()+std::string(".yaml"));
        
        logger_->debug_stream() << "[CameraAdapter::storeImage] Stored image.";
    } 
    return true;
}

bool CameraAdapter::grabPublishImage(cv::Mat& curr_image, ImageMetadata& metadata)
{
    logger_->debug_stream() << "[CameraAdapter::grabPublishImage] Grabbing image.";
    bool result = grabImage(curr_image, metadata);
    if (!result) logger_->error_stream() << "[CameraAdapter::grabPublishImage] Could not grab image from " << getName() << " camera.";
    if (result)
    {
        result = publishImage(curr_image, metadata);
    }
    if (!result) logger_->error_stream() << "[CameraAdapter::grabPublishImage] Could not publish image from " << getName() << " camera.";
    logger_->debug_stream() << "[CameraAdapter::grabPublishImage] Image processed.";
    return result;
}
