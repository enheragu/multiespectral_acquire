#include "utils/image_metadata.h"
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

ImageMetadata::ImageMetadata()
    : camera_timestamp(0),
      ros_timestamp(0),
      trigger_timestamp(0),
      trigger_sent_timestamp(0),
      trigger_ack_timestamp(0),
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

// Init timestamps before sending the trigger
void ImageMetadata::initTimestamps() {
    auto now = std::chrono::steady_clock::now();
    auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
    this->trigger_sent_timestamp = nanos;
    this->timetag = getTimeTag();
    if (ros_time_now_cb_) {
        this->ros_timestamp = ros_time_now_cb_();
    } else {
        this->ros_timestamp = 0;
    }
}

// When trigger finishes updates trigger timestamp to half of it
void ImageMetadata::triggerAck()
{
    auto now = std::chrono::steady_clock::now();
    auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
    this->trigger_ack_timestamp = nanos;
    auto trigger_processing_time = (this->trigger_sent_timestamp - nanos);
    this->trigger_timestamp = this->trigger_sent_timestamp + trigger_processing_time/2;
}

void ImageMetadata::setExposure(uint64_t exposure_ns) {
    this->exposureTime = exposure_ns;
    this->half_exposure_timestamp = this->trigger_timestamp + static_cast<uint64_t>(exposure_ns / 2);
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
    node["ros_timestamp"] = ros_timestamp;
    node["trigger_timestamp"] = trigger_timestamp;
    node["trigger_sent_timestamp"] = trigger_sent_timestamp;
    node["trigger_ack_timestamp"] = trigger_ack_timestamp;
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