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
bool initCamera(int frame_rate);
bool beginAcquisition();
bool endAcquisition();
bool setAsMaster();
bool setAsSlave();
bool acquireImage(cv::Mat& image);
bool closeCamera();


class MultiespectralAcquireT
{
protected:
    std::string img_path = "";
    std::mutex camera_mutex; // Avoid deinitialization while grabbing image

    ros::NodeHandle nh_;
    image_transport::Publisher image_pub_;
public:
    MultiespectralAcquireT(std::string img_path);
    ~MultiespectralAcquireT(void);
    bool init(int frame_rate);
    bool grabStoreImage(cv::Mat& image, bool store = true);
    bool changeFrameRate(int frame_rate);
};

#endif //CAMERA_ADAPTER_H