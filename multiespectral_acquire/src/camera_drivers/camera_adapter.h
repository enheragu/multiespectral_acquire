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

# include "logging_utils.h"

void createTestPattern(cv::Mat& image);

struct ImageMetadata;
void saveMetadataYaml(const ImageMetadata& meta, const std::string& filename);

std::string getName();
std::string getType();
std::string getTimeTag(std::chrono::system_clock::time_point now = std::chrono::system_clock::now());
std::string getFolderTimetag(std::chrono::system_clock::time_point now = std::chrono::system_clock::now());
bool initCamera(int frame_rate, std::string camera_ip);
bool beginAcquisition();
bool endAcquisition();
bool setAsMaster();
bool setAsSlave();
bool acquireImage(cv::Mat& image, ImageMetadata& metadata);
bool closeCamera();

struct ImageMetadata {
    int64_t camera_timestamp;
    int64_t ros_timestamp;
    int64_t system_timestamp;
    int64_t frameCounter;
    double exposureTime;
    double gain;
    int width;
    int height;
    std::string pixelFormat;
    std::string timetag;
    std::string img_name;
    std::string img_pair_name;

    ImageMetadata()
    : camera_timestamp(-1),
      ros_timestamp(-1),
      system_timestamp(-1),
      frameCounter(-1),
      exposureTime(-1.0),
      gain(-1.0),
      width(-1),
      height(-1),
      pixelFormat("UNSET"),
      timetag("UNSET"),
      img_name("UNSET"),
      img_pair_name("UNSET") {}

    void initTimestamps()
    {
        auto now = std::chrono::system_clock::now();
        auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
        this->system_timestamp = nanos;
        this->timetag = getTimeTag(now);
    }

    // Easy access method to choose which timestamp is used
    int64_t getTimestamp() const {
        return system_timestamp;
    }
};

class MultiespectralAcquireT: public rclcpp::Node
{
protected:
    long stored_images = 0;
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
    bool grabImage(cv::Mat& curr_image, ImageMetadata& metadata);
    bool publishImage(cv::Mat& curr_image, ImageMetadata& metadata);
    bool storeImage(cv::Mat& curr_image, ImageMetadata& metadata);
    bool grabPublishImage(cv::Mat& image, ImageMetadata& metadata);
    bool changeFrameRate(int frame_rate);
    
    // see function definition
    void dummyCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

    int getFrameRate() const { return frame_rate; }
};

#endif //CAMERA_ADAPTER_H