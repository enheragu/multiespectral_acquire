/**
 * @file pointcloud_utils.h
 * @brief Utility functions for PointCloud processing.
 *        ROS-agnostic where possible for easy migration between ROS1 and ROS2.
 */
#ifndef POINTCLOUD_UTILS_H
#define POINTCLOUD_UTILS_H

#include <vector>
#include <string>
#include <fstream>
#include <cstring>
#include <algorithm>
#include <cmath>
#include <opencv2/opencv.hpp>

/**
 * @brief FOV configuration for cropping organized pointclouds
 */
struct FOVConfig {
    // FOV limits in columns (horizontal) and rows (vertical) for the organized pointcloud
    // Set to -1 for no cropping on that dimension
    int col_start = -1;  // First column to include (-1 = 0)
    int col_end = -1;    // Last column to include (-1 = width-1)
    int row_start = -1;  // First row to include (-1 = 0)
    int row_end = -1;    // Last row to include (-1 = height-1)
    
    // Alternative: angular FOV (degrees) - if these are set, col_start/end are computed
    double h_fov_min_deg = -180.0;  // Horizontal FOV min (default: full 360)
    double h_fov_max_deg = 180.0;   // Horizontal FOV max
    double v_fov_min_deg = -45.0;   // Vertical FOV min (depends on LiDAR model)
    double v_fov_max_deg = 45.0;    // Vertical FOV max
    
    bool use_angular_fov = false;   // If true, use angular limits instead of pixel limits
    
    /**
     * @brief Compute column indices from angular FOV (for 360° LiDARs like Ouster)
     * @param cloud_width Number of columns in the pointcloud
     */
    void computeColumnsFromAngularFOV(int cloud_width) {
        if (!use_angular_fov) return;
        // Map angle to column index (assuming 360° horizontal FOV, 0° = front)
        double deg_per_col = 360.0 / cloud_width;
        col_start = static_cast<int>((h_fov_min_deg + 180.0) / deg_per_col) % cloud_width;
        col_end = static_cast<int>((h_fov_max_deg + 180.0) / deg_per_col) % cloud_width;
        if (col_start < 0) col_start += cloud_width;
        if (col_end < 0) col_end += cloud_width;
    }
    
    /**
     * @brief Compute row indices from vertical FOV
     * @param cloud_height Number of rows in the pointcloud
     * @param v_fov_total Total vertical FOV of the sensor in degrees
     */
    void computeRowsFromAngularFOV(int cloud_height, double v_fov_total = 45.0) {
        if (!use_angular_fov) return;
        double deg_per_row = v_fov_total / cloud_height;
        row_start = static_cast<int>((v_fov_min_deg + v_fov_total/2) / deg_per_row);
        row_end = static_cast<int>((v_fov_max_deg + v_fov_total/2) / deg_per_row);
        row_start = std::max(0, std::min(row_start, cloud_height - 1));
        row_end = std::max(0, std::min(row_end, cloud_height - 1));
    }
    
    /**
     * @brief Get the actual crop bounds, handling defaults
     */
    void getActualBounds(int original_width, int original_height,
                         int& out_col_start, int& out_col_end,
                         int& out_row_start, int& out_row_end) const {
        out_col_start = (col_start < 0) ? 0 : std::min(col_start, original_width - 1);
        out_col_end = (col_end < 0) ? original_width - 1 : std::min(col_end, original_width - 1);
        out_row_start = (row_start < 0) ? 0 : std::min(row_start, original_height - 1);
        out_row_end = (row_end < 0) ? original_height - 1 : std::min(row_end, original_height - 1);
    }
    
    /**
     * @brief Calculate new dimensions after cropping
     */
    void getCroppedDimensions(int original_width, int original_height,
                              int& new_width, int& new_height) const {
        int cs, ce, rs, re;
        getActualBounds(original_width, original_height, cs, ce, rs, re);
        bool wrap_around = cs > ce;
        new_width = wrap_around ? (original_width - cs + ce + 1) : (ce - cs + 1);
        new_height = re - rs + 1;
    }
};

/**
 * @brief Generic PointCloud2 structure (ROS-agnostic)
 *        Can be converted to/from sensor_msgs::PointCloud2
 */
struct PointCloud2Data {
    uint32_t height = 0;
    uint32_t width = 0;
    uint32_t point_step = 0;
    uint32_t row_step = 0;
    bool is_dense = true;
    bool is_bigendian = false;
    
    struct FieldInfo {
        std::string name;
        uint32_t offset = 0;
        uint8_t datatype = 0;
        uint32_t count = 0;
    };
    std::vector<FieldInfo> fields;
    std::vector<uint8_t> data;
    
    // Timestamp info (stored separately for flexibility)
    uint64_t timestamp_ns = 0;
    uint32_t seq = 0;
    std::string frame_id;
};

/**
 * @brief Save PointCloud2Data to binary file
 * @param cloud The pointcloud data
 * @param filename Output filename
 * @return true if successful
 */
inline bool savePointCloudBin(const PointCloud2Data& cloud, const std::string& filename) {
    std::ofstream ofs(filename, std::ios::binary);
    if (!ofs.is_open()) {
        return false;
    }
    
    // Header info
    ofs.write(reinterpret_cast<const char*>(&cloud.height), sizeof(cloud.height));
    ofs.write(reinterpret_cast<const char*>(&cloud.width), sizeof(cloud.width));
    ofs.write(reinterpret_cast<const char*>(&cloud.point_step), sizeof(cloud.point_step));
    ofs.write(reinterpret_cast<const char*>(&cloud.row_step), sizeof(cloud.row_step));
    uint8_t is_dense = cloud.is_dense ? 1 : 0;
    uint8_t is_bigendian = cloud.is_bigendian ? 1 : 0;
    ofs.write(reinterpret_cast<const char*>(&is_dense), sizeof(is_dense));
    ofs.write(reinterpret_cast<const char*>(&is_bigendian), sizeof(is_bigendian));
    
    // Fields info
    uint32_t num_fields = cloud.fields.size();
    ofs.write(reinterpret_cast<const char*>(&num_fields), sizeof(num_fields));
    for (const auto& field : cloud.fields) {
        uint32_t name_len = field.name.size();
        ofs.write(reinterpret_cast<const char*>(&name_len), sizeof(name_len));
        ofs.write(field.name.data(), name_len);
        ofs.write(reinterpret_cast<const char*>(&field.offset), sizeof(field.offset));
        ofs.write(reinterpret_cast<const char*>(&field.datatype), sizeof(field.datatype));
        ofs.write(reinterpret_cast<const char*>(&field.count), sizeof(field.count));
    }
    
    // Point cloud data
    uint64_t data_size = cloud.data.size();
    ofs.write(reinterpret_cast<const char*>(&data_size), sizeof(data_size));
    ofs.write(reinterpret_cast<const char*>(cloud.data.data()), cloud.data.size());
    ofs.close();
    
    return true;
}

/**
 * @brief Load PointCloud2Data from binary file
 * @param filename Input filename
 * @param cloud Output pointcloud data
 * @return true if successful
 */
inline bool loadPointCloudBin(const std::string& filename, PointCloud2Data& cloud) {
    std::ifstream ifs(filename, std::ios::binary);
    if (!ifs.is_open()) {
        return false;
    }
    
    // Header info
    ifs.read(reinterpret_cast<char*>(&cloud.height), sizeof(cloud.height));
    ifs.read(reinterpret_cast<char*>(&cloud.width), sizeof(cloud.width));
    ifs.read(reinterpret_cast<char*>(&cloud.point_step), sizeof(cloud.point_step));
    ifs.read(reinterpret_cast<char*>(&cloud.row_step), sizeof(cloud.row_step));
    uint8_t is_dense, is_bigendian;
    ifs.read(reinterpret_cast<char*>(&is_dense), sizeof(is_dense));
    ifs.read(reinterpret_cast<char*>(&is_bigendian), sizeof(is_bigendian));
    cloud.is_dense = is_dense != 0;
    cloud.is_bigendian = is_bigendian != 0;
    
    // Fields info
    uint32_t num_fields;
    ifs.read(reinterpret_cast<char*>(&num_fields), sizeof(num_fields));
    cloud.fields.resize(num_fields);
    for (auto& field : cloud.fields) {
        uint32_t name_len;
        ifs.read(reinterpret_cast<char*>(&name_len), sizeof(name_len));
        field.name.resize(name_len);
        ifs.read(&field.name[0], name_len);
        ifs.read(reinterpret_cast<char*>(&field.offset), sizeof(field.offset));
        ifs.read(reinterpret_cast<char*>(&field.datatype), sizeof(field.datatype));
        ifs.read(reinterpret_cast<char*>(&field.count), sizeof(field.count));
    }
    
    // Point cloud data
    uint64_t data_size;
    ifs.read(reinterpret_cast<char*>(&data_size), sizeof(data_size));
    cloud.data.resize(data_size);
    ifs.read(reinterpret_cast<char*>(cloud.data.data()), data_size);
    ifs.close();
    
    return true;
}

/**
 * @brief Crop organized pointcloud data according to FOV config
 * @param cloud Input pointcloud
 * @param fov FOV configuration
 * @return Cropped pointcloud
 */
inline PointCloud2Data cropPointCloud(const PointCloud2Data& cloud, const FOVConfig& fov) {
    int col_start, col_end, row_start, row_end;
    fov.getActualBounds(cloud.width, cloud.height, col_start, col_end, row_start, row_end);
    
    bool wrap_around = col_start > col_end;
    int new_width, new_height;
    fov.getCroppedDimensions(cloud.width, cloud.height, new_width, new_height);
    
    PointCloud2Data cropped;
    cropped.height = new_height;
    cropped.width = new_width;
    cropped.fields = cloud.fields;
    cropped.is_bigendian = cloud.is_bigendian;
    cropped.point_step = cloud.point_step;
    cropped.row_step = new_width * cloud.point_step;
    cropped.is_dense = cloud.is_dense;
    cropped.timestamp_ns = cloud.timestamp_ns;
    cropped.seq = cloud.seq;
    cropped.frame_id = cloud.frame_id;
    cropped.data.resize(new_height * new_width * cloud.point_step);
    
    size_t dst_idx = 0;
    for (int row = row_start; row <= row_end; ++row) {
        if (wrap_around) {
            // First part: from col_start to end of original row
            size_t src_offset = row * cloud.row_step + col_start * cloud.point_step;
            size_t bytes_to_copy = (cloud.width - col_start) * cloud.point_step;
            std::memcpy(&cropped.data[dst_idx], &cloud.data[src_offset], bytes_to_copy);
            dst_idx += bytes_to_copy;
            
            // Second part: from start of row to col_end
            src_offset = row * cloud.row_step;
            bytes_to_copy = (col_end + 1) * cloud.point_step;
            std::memcpy(&cropped.data[dst_idx], &cloud.data[src_offset], bytes_to_copy);
            dst_idx += bytes_to_copy;
        } else {
            size_t src_offset = row * cloud.row_step + col_start * cloud.point_step;
            size_t bytes_to_copy = new_width * cloud.point_step;
            std::memcpy(&cropped.data[dst_idx], &cloud.data[src_offset], bytes_to_copy);
            dst_idx += bytes_to_copy;
        }
    }
    
    return cropped;
}

/**
 * @brief Extract intensity image from organized pointcloud
 * @param cloud Input pointcloud
 * @param intensity_img Output intensity image (CV_8UC1)
 * @param fov FOV configuration for cropping
 * @return true if intensity field was found and image extracted
 */
inline bool extractIntensityImage(const PointCloud2Data& cloud, cv::Mat& intensity_img, 
                                  const FOVConfig& fov) {
    // Find intensity field
    auto it = std::find_if(cloud.fields.begin(), cloud.fields.end(),
        [](const PointCloud2Data::FieldInfo& f) { return f.name == "intensity"; });
    if (it == cloud.fields.end()) return false;
    
    int offset = it->offset;
    size_t point_step = cloud.point_step;
    
    int col_start, col_end, row_start, row_end;
    fov.getActualBounds(cloud.width, cloud.height, col_start, col_end, row_start, row_end);
    
    bool wrap_around = col_start > col_end;
    int new_width, new_height;
    fov.getCroppedDimensions(cloud.width, cloud.height, new_width, new_height);
    
    cv::Mat float_img = cv::Mat::zeros(new_height, new_width, CV_32FC1);
    
    for (int dst_row = 0; dst_row < new_height; ++dst_row) {
        int src_row = row_start + dst_row;
        int dst_col = 0;
        
        auto processColumn = [&](int src_col) {
            size_t idx = src_row * cloud.width + src_col;
            if (idx * point_step + offset + 4 <= cloud.data.size()) {
                float intensity = *reinterpret_cast<const float*>(&cloud.data[idx * point_step + offset]);
                float_img.at<float>(dst_row, dst_col) = intensity;
            }
            ++dst_col;
        };
        
        if (wrap_around) {
            for (int src_col = col_start; src_col < static_cast<int>(cloud.width); ++src_col)
                processColumn(src_col);
            for (int src_col = 0; src_col <= col_end; ++src_col)
                processColumn(src_col);
        } else {
            for (int src_col = col_start; src_col <= col_end; ++src_col)
                processColumn(src_col);
        }
    }
    
    // Normalize to 8-bit
    double minVal, maxVal;
    cv::minMaxLoc(float_img, &minVal, &maxVal);
    if (maxVal > minVal) {
        float_img.convertTo(intensity_img, CV_8UC1, 255.0/(maxVal-minVal), -minVal*255.0/(maxVal-minVal));
    } else {
        float_img.convertTo(intensity_img, CV_8UC1);
    }
    
    return true;
}

#endif // POINTCLOUD_UTILS_H
