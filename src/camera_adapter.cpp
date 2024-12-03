

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
    ROS_FATAL_STREAM_COND(!result, "[MultiespectralAcquireT] Could not initialize " << getName() << " camera.");
    return result;
}

bool MultiespectralAcquireT::changeFrameRate(int frame_rate)
{
    ROS_INFO_STREAM("[MultiespectralAcquireT::changeFrameRate] Closing camera to setup new frame rate to: " << frame_rate);
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
    ROS_FATAL_STREAM_COND(!result, "[MultiespectralAcquireT] Could not finish correctly " << getName() << " camera.");
    ROS_INFO_STREAM_COND(result, "[MultiespectralAcquireT] Correctly finished " << getName() << " camera.");
}

bool MultiespectralAcquireT::grabStoreImage(cv::Mat& curr_image, bool store)
{

    const std::scoped_lock<std::mutex> lock(camera_mutex);
    bool result =  acquireImage(curr_image);
    if (result && !curr_image.empty() and store) 
    {
        std::ostringstream filename;
        filename << img_path << "/" << getTimeTag() << ".jpg";
        cv::imwrite(filename.str().c_str(), curr_image);
    }
    
    // Convert to a sensor_msgs::Image message detecting encoding
    std::string encoding;
    if (curr_image.type() == CV_8UC3) {
        encoding = sensor_msgs::image_encodings::BGR8;
    } else if (curr_image.type() == CV_8UC1) {
        encoding = sensor_msgs::image_encodings::MONO8;
    } else {
        std::cerr << "Unsupported image type: " << curr_image.type() << std::endl;
        return false;
    }
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), encoding, curr_image).toImageMsg();
    image_pub_.publish(msg);

    ROS_ERROR_STREAM_COND(!result, "[MultiespectralAcquireT::grabStoreImage] Could not acquire image from " << getName() << " camera.");
    return result;
}

