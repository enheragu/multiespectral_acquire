/**
 * Header for basler_adapter.cpp and flir_adapter.cpp
 */
#ifndef CAMERA_ADAPTER_H
#define CAMERA_ADAPTER_H


#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

std::string getName();
std::string getType();
bool initCamera(int frame_rate, std::string camera_ip);
bool beginAcquisition();
bool endAcquisition();
bool setAsMaster();
bool setAsSlave();
bool acquireImage(cv::Mat& image, uint64_t& timestamp);
bool closeCamera();


class MultiespectralAcquireT
{
protected:
    std::string img_path = "";
    std::mutex camera_mutex; // Avoid deinitialization while grabbing image

    std::string camera_ip;
    std::string topic_name;
    
    ros::NodeHandle nh_;
    image_transport::Publisher image_pub_;
public:
    MultiespectralAcquireT(std::string img_path, std::string topic_name);
    ~MultiespectralAcquireT(void);
    bool init(int frame_rate, std::string camera_ip);
    bool grabImage(cv::Mat& curr_image, uint64_t& timestamp);
    bool StoreImage(cv::Mat& curr_image, uint64_t& timestamp, bool store = true);
    bool grabStoreImage(cv::Mat& image, uint64_t& timestamp, bool store = true);
    bool changeFrameRate(int frame_rate);
    
    // see function definition
    void dummyCallback(const sensor_msgs::ImageConstPtr& msg);
};

#endif //CAMERA_ADAPTER_H