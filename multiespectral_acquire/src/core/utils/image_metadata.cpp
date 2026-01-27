#include "core/utils/image_metadata.h"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <chrono>
#include <ctime>
#include <iomanip>

std::string getFolderTimetag() {
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

std::string getUTCTimeTag() {
    auto now = std::chrono::system_clock::now();
    std::time_t utc = std::chrono::system_clock::to_time_t(now);
    std::tm* tm_utc = std::gmtime(&utc);  // UTC!
    
    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    
    std::ostringstream oss;
    oss << std::put_time(tm_utc, "%H%M%S") 
        << '.' << std::setfill('0') << std::setw(3) << millis.count();
    return oss.str();
}

ImageMetadata::ImageMetadata()
    : camera_timestamp(0),
      exposureTime(0),
      half_exposure_timestamp(0),
      frameCounter(0),
      gain(-1.0),
      width(-1),
      height(-1),
      pixelFormat("UNSET"),
      timetag("UNSET"),
      img_name("UNSET"),
      img_pair_name("UNSET"),
      dataset_name("UNSET"),
      ros_time_now_cb_(nullptr) {}

void ImageMetadata::setROSTimeNowCallback(ROSTimeNowCallback cb) {
    ros_time_now_cb_ = cb;
}

void ImageMetadata::setExposure(uint64_t exposure_ns) {
    this->exposureTime = exposure_ns;
    // Half exposure timestamp = camera timestamp + half of exposure duration
    this->half_exposure_timestamp = this->camera_timestamp + static_cast<uint64_t>(exposure_ns / 2);
}

uint64_t ImageMetadata::getSyncTimestamp() const {
    return half_exposure_timestamp;
}

void ImageMetadata::saveYaml(const std::string& filename) const {
    YAML::Node node;
    node["dataset_name"] = dataset_name;
    node["img_name"] = img_name;
    node["timetag"] = timetag;
    node["camera_timestamp"] = camera_timestamp;
    node["half_exposure_timestamp"] = half_exposure_timestamp;
    node["frameCounter"] = frameCounter;
    node["exposureTime"] = exposureTime;
    node["gain"] = gain;
    node["width"] = width;
    node["height"] = height;
    node["pixelFormat"] = pixelFormat;
    node["img_pair_name"] = img_pair_name;
    std::ofstream fout(filename);
    fout << node;
}