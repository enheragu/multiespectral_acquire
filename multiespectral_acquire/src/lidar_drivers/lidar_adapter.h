// LiDAR adapter base class and utilities (ROS-agnostic)
// Analogous to camera_drivers/camera_adapter.h
#ifndef LIDAR_ADAPTER_H
#define LIDAR_ADAPTER_H

#include <string>
#include <iostream>
#include <deque>
#include <mutex>
#include <filesystem>
#include <opencv2/opencv.hpp>

#include "utils/pointcloud_utils.h"
#include "utils/image_metadata.h"
#include "utils/logging_utils.h"
#include "utils/timed_frame_buffer.h"

// Frame data for buffering (pointcloud + intensity image + metadata)
struct LidarFrameData {
    uint64_t timestamp = 0;
    PointCloud2Data pointcloud;
    cv::Mat intensity_image;
    ImageMetadata pc_metadata;
    ImageMetadata intensity_metadata;
    GNSSMetadata gnss_data;
    OdomMetadata odom_data;
    

    void addGNSSData(GNSSMetadata gnss) { gnss_data = gnss; }
    void addOdomData(OdomMetadata odom) { odom_data = odom; }
    
    void fillMetadata(uint64_t timestamp_ns, uint32_t seq, 
                      const std::string& dataset_name, int frame_rate) {
        fillCommonMetadata(pc_metadata, timestamp_ns, seq, dataset_name, frame_rate);
        pc_metadata.width = pointcloud.width;
        pc_metadata.height = pointcloud.height;
        pc_metadata.pixelFormat = "PointCloud2";
        
        fillCommonMetadata(intensity_metadata, timestamp_ns, seq, dataset_name, frame_rate);
        intensity_metadata.width = intensity_image.cols;
        intensity_metadata.height = intensity_image.rows;
        intensity_metadata.pixelFormat = "CV_8UC1";
        
        timestamp = pc_metadata.getSyncTimestamp();
    }
    
    void setReferencePair(const std::string& reference_pair) {
        pc_metadata.img_pair_name = reference_pair;
        intensity_metadata.img_pair_name = reference_pair;
    }
    
    bool save(const std::string& pc_path, const std::string& intensity_path, 
              const std::string& base_name) const {
        bool success = savePointCloudBin(pointcloud, pc_path + base_name + ".bin");
        pc_metadata.saveYaml(pc_path + base_name + ".yaml");
        
        if (!intensity_image.empty()) {
            cv::imwrite(intensity_path + base_name + ".png", intensity_image);
            intensity_metadata.saveYaml(intensity_path + base_name + ".yaml");
        }
        return success;
    }
};

inline void processPointCloud(const PointCloud2Data& cloud, LidarFrameData& frame,
                              FOVConfig& fov_config, bool apply_fov) {
    if (apply_fov) {
        if (fov_config.use_angular_fov) {
            fov_config.computeColumnsFromAngularFOV(cloud.width);
            fov_config.computeRowsFromAngularFOV(cloud.height, 45.0);
        }
        frame.pointcloud = cropPointCloud(cloud, fov_config);
        extractIntensityImage(cloud, frame.intensity_image, fov_config);
    } else {
        frame.pointcloud = cloud;
        FOVConfig no_crop;
        extractIntensityImage(cloud, frame.intensity_image, no_crop);
    }
}

// LiDAR adapter base class (ROS-agnostic). Manages paths, buffering, storage.
class LidarAdapterSlave {
protected:
    std::shared_ptr<Logger> logger_;
    
    TimedFrameBuffer<LidarFrameData> buffer_;
    std::mutex buffer_mutex_;
    
    int sensor_frame_rate_ = 20;      // Real sensor rate (hardware)
    int configured_frame_rate_ = 10;  // Subsampled/configured rate
    
    std::string dataset_output_path_ = ".";
    std::string pc_path_;
    std::string intensity_path_;
    std::string dataset_name_;
    
    FOVConfig fov_config_;
    bool fov_enabled_ = false;

public:
    virtual ~LidarAdapterSlave() = default;
    
    bool init_store_folder(const std::string& output_dataset_path) {
        dataset_output_path_ = output_dataset_path;
        dataset_name_ = getFolderTimetag();
        
        std::string folder = dataset_output_path_ + "/" + dataset_name_ + "/";
        pc_path_ = folder + "PointCloud/";
        intensity_path_ = folder + "swir/";
        
        try {
            std::filesystem::create_directories(pc_path_);
            std::filesystem::create_directories(intensity_path_);
        } catch (const std::exception& e) {
            if (logger_) logger_->error_stream() << "[LidarAdapterSlave::init_store_folder] Failed to create directories: " << e.what();
            return false;
        }
        
        if (logger_) logger_->info_stream() << "[LidarAdapterSlave::init_store_folder] Storage initialized: " << folder;
        return true;
    }
    
    void setLogger(std::shared_ptr<Logger> logger) { logger_ = logger; }
};

#endif // LIDAR_ADAPTER_H
