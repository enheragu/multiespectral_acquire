/**
 * Header for basler_adapter.cpp and flir_adapter.cpp
 */
#ifndef CAMERA_ADAPTER_H
#define CAMERA_ADAPTER_H

#include <yaml-cpp/yaml.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <sensor_msgs/msg/image.hpp>
#include "cv_bridge/cv_bridge.hpp"
#include "image_transport/image_transport.hpp"

#include <camera_info_manager/camera_info_manager.hpp>
#include <sensor_msgs/msg/camera_info.hpp>


void createTestPattern(cv::Mat& image);

struct ImageMetadata;
void saveMetadataYaml(const ImageMetadata& meta, const std::string& filename);

std::string getName();
std::string getType();
std::string getTimeTag();
std::string getFolderTimetag();
bool initCamera(int frame_rate, std::string camera_ip);
bool beginAcquisition();
bool endAcquisition();
bool setAsMaster();
bool setAsSlave();
bool acquireImage(cv::Mat& image, uint64_t& timestamp, ImageMetadata& metadata);
bool closeCamera();

struct ImageMetadata {
    int64_t timestamp;
    int64_t frameCounter;
    double exposureTime;
    double gain;
    int width;
    int height;
    std::string pixelFormat;
    std::string systemTime;

    ImageMetadata()
    : timestamp(-1),
      frameCounter(-1),
      exposureTime(-1.0),
      gain(-1.0),
      width(-1),
      height(-1),
      pixelFormat("UNSET"),
      systemTime("UNSET") {}
};

class MultiespectralAcquireT: public rclcpp::Node
{
protected:
    int frame_rate = 5;

    std::string img_path = "";
    std::mutex camera_mutex; // Avoid deinitialization while grabbing image

    std::string camera_ip;
    std::string topic_name;

    std::string camera_info_cfg;
    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;
    
    rclcpp::Node::SharedPtr node_handle_;
    image_transport::ImageTransport it_;    
    image_transport::CameraPublisher image_pub_;

public:
    MultiespectralAcquireT(std::string name);
    ~MultiespectralAcquireT(void);
    bool init(int frame_rate);
    bool grabImage(cv::Mat& curr_image, uint64_t& timestamp, ImageMetadata& metadata);
    bool processImage(cv::Mat& curr_image, uint64_t& timestamp, ImageMetadata& metadata, bool store = true);
    bool grabStoreImage(cv::Mat& image, uint64_t& timestamp, ImageMetadata& metadata, bool store = true);
    bool changeFrameRate(int frame_rate);
    
    // see function definition
    void dummyCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

    int getFrameRate() const { return frame_rate; }
};

#endif //CAMERA_ADAPTER_H