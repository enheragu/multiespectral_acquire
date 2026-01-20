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
    void setExposure(uint64_t exposure_ns);
    uint64_t getSyncTimestamp() const;
    void saveYaml(const std::string& filename) const;

public:
    ROSTimeNowCallback ros_time_now_cb_;
    uint64_t camera_timestamp;        // all in nanoseconds for compatibility
    uint64_t ros_timestamp;           // all in nanoseconds for compatibility
    uint64_t trigger_timestamp;       // all in nanoseconds for compatibility
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
};

// Utility function declarations for image metadata
std::string getTimeTag();
std::string getFolderTimetag();
void saveMetadataYaml(const ImageMetadata& meta, const std::string& filename);

#endif // IMAGE_METADATA_H