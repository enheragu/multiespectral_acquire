/**
 * Header for for camera_adapter.cpp that merges both basler_adapter.cpp and flir_adapter.cpp into a compatible object
 */
#ifndef CAMERA_ADAPTER_H
#define CAMERA_ADAPTER_H

#include <string>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include "logging_utils.h"
#include "utils/image_metadata.h"

void createTestPattern(cv::Mat& image);

std::string getName();
std::string getType();
bool initCamera(int frame_rate, std::string camera_ip);
bool beginAcquisition();
bool endAcquisition();
bool setAsMaster();
bool setAsSlave();
bool acquireImage(cv::Mat& image, ImageMetadata& metadata);
bool closeCamera();

class CameraAdapter {
protected:
    std::shared_ptr<Logger> logger_;

    long stored_images = 0;
    int frame_rate = 5;

    std::string img_path = "";
    std::mutex camera_mutex; // Avoid deinitialization while grabbing image

    std::string camera_ip;
    std::string camera_info_cfg;

public:
    ~CameraAdapter(void);
    bool init(int frame_rate);
    bool grabImage(cv::Mat& curr_image, ImageMetadata& metadata);
    virtual bool publishImage(cv::Mat& curr_image, ImageMetadata& metadata) {
        std::cerr << "[CameraAdapter] publishImage() called on core adapter. This should be implemented in a middleware-specific subclass (e.g., CameraAdapterROS)." << std::endl;
        return false;
    }
    bool grabPublishImage(cv::Mat& image, ImageMetadata& metadata);
    bool storeImage(cv::Mat& curr_image, ImageMetadata& metadata);
    bool grabPublishImage(cv::Mat& image, ImageMetadata& metadata);
    bool changeFrameRate(int frame_rate);
    
    // see function definition
    void dummyCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

    int getFrameRate() const { return frame_rate; }

    void setLogger(std::shared_ptr<Logger> logger) { logger_ = logger; }

};

#endif //CAMERA_ADAPTER_H