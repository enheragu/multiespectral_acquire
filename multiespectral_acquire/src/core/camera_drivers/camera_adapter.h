/**
 * Header for for camera_adapter.cpp that merges both basler_adapter.cpp and flir_adapter.cpp into a compatible object
 */
#ifndef CAMERA_ADAPTER_H
#define CAMERA_ADAPTER_H

#include <string>
#include <iostream>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "core/utils/logging_utils.h"
#include "core/utils/image_metadata.h"

void createTestPattern(cv::Mat& image);

void setNodeName(const std::string& name);
std::string getName();
std::string getModelName();
std::string getType();

// External variables to control PTP behavior (set before initCamera)
extern bool force_disable_ptp;

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

    std::string dataset_name;

public:
    ~CameraAdapter(void);
    bool init(int frame_rate);
    bool init_store_folder(std::string output_dataset_path);
    bool grabImage(cv::Mat& curr_image, ImageMetadata& metadata);
    virtual bool publishImage(cv::Mat& curr_image, ImageMetadata& metadata) {
        std::cerr << "[CameraAdapter] publishImage() called on core adapter. This should be implemented in a middleware-specific subclass (e.g., CameraAdapterROS)." << std::endl;
        return false;
    }
    bool grabPublishImage(cv::Mat& image, ImageMetadata& metadata);
    bool storeImage(cv::Mat& curr_image, ImageMetadata& metadata);
    bool changeFrameRate(int frame_rate);
    
    int getFrameRate() const { return frame_rate; }

    void setLogger(std::shared_ptr<Logger> logger) { logger_ = logger; }

};

#endif //CAMERA_ADAPTER_H