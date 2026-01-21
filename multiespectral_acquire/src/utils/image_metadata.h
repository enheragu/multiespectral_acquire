#ifndef IMAGE_METADATA_H
#define IMAGE_METADATA_H
#include <string>
#include <chrono>
#include <functional>

class ImageMetadata {
public:
    using ROSTimeNowCallback = std::function<uint64_t()>;

    ImageMetadata();

    void setROSTimeNowCallback(ROSTimeNowCallback cb);
    void initTimestamps();
    void triggerAck();
    void setExposure(uint64_t exposure_ns);
    uint64_t getSyncTimestamp() const;
    void saveYaml(const std::string& filename) const;

public:
    ROSTimeNowCallback ros_time_now_cb_;
    uint64_t camera_timestamp;        // all in nanoseconds for compatibility
    uint64_t ros_timestamp;           // all in nanoseconds for compatibility
    uint64_t trigger_timestamp;      // all in nanoseconds for compatibility
    uint64_t trigger_sent_timestamp;   // all in nanoseconds for compatibility
    uint64_t trigger_ack_timestamp;    // all in nanoseconds for compatibility
    uint64_t exposureTime;            // all in nanoseconds for compatibility
    uint64_t half_exposure_timestamp; // nanoseconds (estimated)
    uint64_t frameCounter;
    double gain;
    int width;
    int height;
    std::string pixelFormat;
    std::string timetag;
    std::string img_name;
    std::string img_pair_name;
    std::string dataset_name;
};

// Utility function declarations for image metadata
std::string getTimeTag();
std::string getFolderTimetag();
void saveMetadataYaml(const ImageMetadata& meta, const std::string& filename);


/**
 * @brief Fill common fields in ImageMetadata from timestamp info
 */
inline void fillCommonMetadata(ImageMetadata& meta, 
                               uint64_t timestamp_ns, 
                               uint32_t seq, 
                               const std::string& dataset_name,
                               int frame_rate) {
    meta.ros_timestamp = timestamp_ns;
    meta.camera_timestamp = timestamp_ns;
    meta.trigger_timestamp = timestamp_ns;
    meta.dataset_name = dataset_name;
    meta.frameCounter = seq;
    
    // Calculate timetag from timestamp
    std::chrono::nanoseconds ns_since_epoch(timestamp_ns);
    std::chrono::system_clock::time_point tp(ns_since_epoch);
    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch()) % 1000;
    std::time_t timeNow = std::chrono::system_clock::to_time_t(tp);
    std::tm* tmNow = std::localtime(&timeNow);
    std::ostringstream oss;
    oss << std::put_time(tmNow, "%y-%m-%d_%H-%M-%S_") << std::setw(3) << std::setfill('0') << millis.count();
    meta.timetag = oss.str();
    
    // Set exposure based on frame rate
    if (frame_rate > 0) {
        meta.setExposure(static_cast<uint64_t>(1e9 / frame_rate));
    }
}

#endif // IMAGE_METADATA_H