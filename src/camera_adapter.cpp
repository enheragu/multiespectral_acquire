

#include <mutex>
#include <iostream>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

#include "ros/ros.h"

#include "camera_adapter.h"


std::string getTimeTag() {
    auto now = std::chrono::system_clock::now();
    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    std::time_t timeNow = std::chrono::system_clock::to_time_t(now);
    
    std::tm* tmNow = std::localtime(&timeNow);

    std::ostringstream oss;
    oss << std::put_time(tmNow, "%y-%m-%d_%H-%M-%S_") << std::setw(3) << std::setfill('0') << millis.count();

    return oss.str();
}

bool MultiespectralAcquireT::init(int frame_rate)
{
    bool result = initCamera(frame_rate);
    ROS_FATAL_STREAM_COND(!result, "[MAT] Could not initialize " << getName() << " camera.");
    return result;
}

bool MultiespectralAcquireT::changeFrameRate(int frame_rate)
{
    ROS_INFO_STREAM("[MAT::changeFrameRate] Closing camera to setup new frame rate to: " << frame_rate);
    bool result = closeCamera();
    this->init(frame_rate);
}


MultiespectralAcquireT::MultiespectralAcquireT(std::string img_path) : img_path(img_path)
{
    image_transport::ImageTransport it(nh_);
    image_pub_ = it.advertise(getType()+"_image", 1);
}

MultiespectralAcquireT::~MultiespectralAcquireT(void)
{   
    const std::scoped_lock<std::mutex> lock(camera_mutex);
    bool result = closeCamera();
    ROS_FATAL_STREAM_COND(!result, "[MAT] Could not finish correctly " << getName() << " camera.");
    ROS_INFO_STREAM_COND(result, "[MAT] Correctly finished " << getName() << " camera.");
}

bool MultiespectralAcquireT::grabImage(cv::Mat& curr_image, uint64_t& timestamp)
{
    const std::scoped_lock<std::mutex> lock(camera_mutex);
    bool result =  acquireImage(curr_image, timestamp);

    // TBD use camera timestamp once synchronized
    ros::Time now = ros::Time::now();
    timestamp = now.sec * 1e9 + now.nsec; // Convertir a nanosegundos

    ROS_ERROR_STREAM_COND(!result, "[MAT::grabImage] Could not acquire image from " << getName() << " camera.");
    return result;
}

bool MultiespectralAcquireT::StoreImage(cv::Mat& curr_image, uint64_t& timestamp, bool store)
{
    const std::scoped_lock<std::mutex> lock(camera_mutex);
    if (!curr_image.empty() && store) 
    {
        std::ostringstream filename;
        filename << img_path << "/" << getTimeTag() << "_tcam_" << timestamp << ".png";
        cv::imwrite(filename.str().c_str(), curr_image);
    } 
    
    // Convert to a sensor_msgs::Image message detecting encoding
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
        // ROS_INFO_STREAM("[MAT::StoreImage] Got image from "<<getName()<<", store with timestamp ("<<timestamp<<") and publish it.");
        std_msgs::Header header;
        // Convertir timestamp_ns a segundos y nanosegundos 
        uint64_t sec = timestamp / 1000000000; 
        uint64_t nsec = timestamp % 1000000000;
        header.stamp = timestamp == 0 ? ros::Time::now() : ros::Time(sec, nsec);

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, encoding, curr_image).toImageMsg();
        image_pub_.publish(msg);
        ros::spinOnce(); // without explicit spinOnce, the LWIR image is as black (rgb is ok...). The info stream also works?¿
        ROS_INFO_STREAM("[MAT::StoreImage] Published image from "<<getName()<<" with encoding: " << encoding);
    }

    ROS_ERROR_STREAM_COND(curr_image.empty(), "[MAT::StoreImage] Image is empty for " << getName() << " camera.");
    return true;
}

bool MultiespectralAcquireT::grabStoreImage(cv::Mat& curr_image, uint64_t& timestamp, bool store)
{
    // ROS_INFO("[MAT::grabStoreImage] Grabbing image.");
    bool result =  grabImage(curr_image, timestamp);
    // ROS_INFO("[MAT::grabStoreImage] Storing image.");
    result = result && StoreImage(curr_image, timestamp, store);
    // ROS_INFO("[MAT::grabStoreImage] Image stored.");
    return result;
}

