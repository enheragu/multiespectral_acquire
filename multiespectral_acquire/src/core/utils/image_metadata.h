#ifndef IMAGE_METADATA_H
#define IMAGE_METADATA_H

#include <string>
#include <iomanip>
#include <chrono>
#include <functional>
#include <yaml-cpp/yaml.h>

struct GNSSMetadata {
    float latitude;
    float longitude;
    float altitude;
    std::vector<float> position_covariance;
    GNSSMetadata() : latitude(0.0), longitude(0.0), altitude(0.0), position_covariance(9, 0.0f) {    }
    GNSSMetadata(float lat, float lon, float alt, std::vector<float> cov) : 
            latitude(lat), longitude(lon), altitude(alt), position_covariance(cov) {    }
    
    YAML::Node toYAMLNode() const {
        YAML::Node node;
        node["latitude"] = latitude;
        node["longitude"] = longitude;
        node["altitude"] = altitude;
        YAML::Node cov_node;
        for (int i = 0; i < 9; i++) {
            cov_node.push_back(position_covariance[i]);
        }
        node["position_covariance"] = cov_node;
        return node;
    }
};

struct OdomMetadata {
    std::vector<float> pos_xyz;
    std::vector<float> ori_xyzw;
    std::vector<float> pose_covariance;
    OdomMetadata() : pos_xyz{0.0,0.0,0.0}, ori_xyzw{0.0,0.0,0.0,1.0}, pose_covariance(36, 0.0f) {    }
    OdomMetadata(float x, float y, float z, float ox, float oy, float oz, float ow, std::vector<float> cov) :
            pos_xyz{x,y,z}, ori_xyzw{ox,oy,oz,ow}, pose_covariance(cov) {    }
    
    YAML::Node toYAMLNode() const {
        YAML::Node node;
        node["pos_xyz"] = pos_xyz;
        node["ori_xyzw"] = ori_xyzw;
        YAML::Node cov_node;
        for (int i = 0; i < 36; i++) {
            cov_node.push_back(pose_covariance[i]);
        }
        node["pose_covariance"] = cov_node;
        return node;
    }
};

class ImageMetadata {
public:
    using ROSTimeNowCallback = std::function<uint64_t()>;

    ImageMetadata();

    void updateTimetag();
    void setROSTimeNowCallback(ROSTimeNowCallback cb);
    void setExposure(uint64_t exposure_ns);
    uint64_t getSyncTimestamp() const;
    void saveYaml(const std::string& filename) const;
    void addGNSSData(GNSSMetadata gnss) { gnss_data = gnss; }
    void addOdomData(OdomMetadata odom) { odom_data = odom; }

public:
    ROSTimeNowCallback ros_time_now_cb_;
    uint64_t camera_timestamp;        // Hardware timestamp (PTP or calibrated), nanoseconds
    uint64_t exposureTime;            // Exposure duration, nanoseconds
    uint64_t half_exposure_timestamp; // camera_timestamp + exposure/2 (sync point), nanoseconds
    uint64_t frameCounter;
    double gain;
    int width;
    int height;
    std::string pixelFormat;
    std::string timetag;
    std::string img_name;
    std::string img_pair_name;
    std::string dataset_name;

    GNSSMetadata gnss_data;
    OdomMetadata odom_data;
};

// Utility function declarations for image metadata
std::string getTimeTag();
std::string getFolderTimetag();


/**
 * @brief Fill common fields in ImageMetadata from timestamp info
 */
inline void fillCommonMetadata(ImageMetadata& meta, 
                               uint64_t timestamp_ns, 
                               uint32_t seq, 
                               const std::string& dataset_name,
                               int frame_rate) {
    meta.camera_timestamp = timestamp_ns;
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