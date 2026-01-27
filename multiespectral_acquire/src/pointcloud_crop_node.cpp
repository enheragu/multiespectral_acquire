/**
 * @file    pointcloud_crop_node.cpp
 * @brief   Crops pointcloud by FOV and republishes to _cropped topic
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <cmath>

class PointCloudCropNode {
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    
    ros::Subscriber sub_pointcloud_;
    ros::Publisher pub_pointcloud_;
    
    std::string input_topic_;
    std::string output_topic_;
    
    bool fov_enabled_;
    bool use_angular_;
    int col_start_, col_end_;
    int row_start_, row_end_;
    double h_fov_min_deg_, h_fov_max_deg_;
    double v_fov_min_deg_, v_fov_max_deg_;
    bool fov_computed_;
    
public:
    PointCloudCropNode() : nh_(), pnh_("~"), fov_computed_(false) {
        // Parameters
        pnh_.param<std::string>("input_topic", input_topic_, "/ouster/points");
        
        // Output topic is always input + "_cropped"
        output_topic_ = input_topic_ + "_cropped";
        
        pnh_.param<bool>("fov_enabled", fov_enabled_, false);
        
        if (fov_enabled_) {
            pnh_.param<bool>("fov_use_angular", use_angular_, false);
            if (use_angular_) {
                pnh_.param<double>("fov_h_min_deg", h_fov_min_deg_, -45.0);
                pnh_.param<double>("fov_h_max_deg", h_fov_max_deg_, 45.0);
                pnh_.param<double>("fov_v_min_deg", v_fov_min_deg_, -22.5);
                pnh_.param<double>("fov_v_max_deg", v_fov_max_deg_, 22.5);
                ROS_INFO("[PointCloudCropNode] Angular FOV: H[%.1f, %.1f], V[%.1f, %.1f]",
                         h_fov_min_deg_, h_fov_max_deg_, v_fov_min_deg_, v_fov_max_deg_);
            } else {
                pnh_.param<int>("fov_col_start", col_start_, 0);
                pnh_.param<int>("fov_col_end", col_end_, -1);
                pnh_.param<int>("fov_row_start", row_start_, 0);
                pnh_.param<int>("fov_row_end", row_end_, -1);
                ROS_INFO("[PointCloudCropNode] Pixel FOV: cols[%d,%d], rows[%d,%d]",
                         col_start_, col_end_, row_start_, row_end_);
            }
        }
        
        // Publishers/Subscribers
        pub_pointcloud_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 1);
        sub_pointcloud_ = nh_.subscribe(input_topic_, 10, &PointCloudCropNode::pointcloud_cb, this);
        
        ROS_INFO("[PointCloudCropNode] Initialized. %s -> %s, FOV: %s",
                 input_topic_.c_str(), output_topic_.c_str(), fov_enabled_ ? "YES" : "NO");
    }
    
    void computePixelFOV(int width, int height) {
        if (fov_computed_) return;
        
        // Ouster: 360° horizontal, ~45° vertical
        double total_h_fov = 360.0;
        double total_v_fov = 45.0;
        
        col_start_ = static_cast<int>((h_fov_min_deg_ + total_h_fov / 2.0) / total_h_fov * width);
        col_end_ = static_cast<int>((h_fov_max_deg_ + total_h_fov / 2.0) / total_h_fov * width);
        row_start_ = static_cast<int>((total_v_fov / 2.0 - v_fov_max_deg_) / total_v_fov * height);
        row_end_ = static_cast<int>((total_v_fov / 2.0 - v_fov_min_deg_) / total_v_fov * height);
        
        col_start_ = std::max(0, std::min(col_start_, width - 1));
        col_end_ = std::max(0, std::min(col_end_, width - 1));
        row_start_ = std::max(0, std::min(row_start_, height - 1));
        row_end_ = std::max(0, std::min(row_end_, height - 1));
        
        ROS_INFO("[PointCloudCropNode] Computed pixel FOV: cols[%d,%d], rows[%d,%d]",
                 col_start_, col_end_, row_start_, row_end_);
        fov_computed_ = true;
    }
    
    void pointcloud_cb(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        if (!fov_enabled_) {
            // No crop, just republish
            pub_pointcloud_.publish(msg);
            return;
        }
        
        // Compute pixel FOV from angular on first message
        if (use_angular_ && !fov_computed_) {
            computePixelFOV(msg->width, msg->height);
        }
        
        // Validate ranges
        if (col_end_ < 0) col_end_ = msg->width - 1;
        if (row_end_ < 0) row_end_ = msg->height - 1;
        
        if (col_start_ >= col_end_ || row_start_ >= row_end_ ||
            col_end_ >= static_cast<int>(msg->width) || row_end_ >= static_cast<int>(msg->height)) {
            ROS_WARN_THROTTLE(5.0, "[PointCloudCropNode] Invalid FOV bounds, publishing original");
            pub_pointcloud_.publish(msg);
            return;
        }
        
        // Crop the pointcloud
        sensor_msgs::PointCloud2 cropped;
        cropped.header = msg->header;
        cropped.height = row_end_ - row_start_;
        cropped.width = col_end_ - col_start_;
        cropped.fields = msg->fields;
        cropped.is_bigendian = msg->is_bigendian;
        cropped.point_step = msg->point_step;
        cropped.row_step = cropped.width * msg->point_step;
        cropped.is_dense = msg->is_dense;
        
        // Copy data row by row
        cropped.data.resize(cropped.height * cropped.row_step);
        for (int row = 0; row < static_cast<int>(cropped.height); ++row) {
            int src_row = row_start_ + row;
            const uint8_t* src_ptr = &msg->data[src_row * msg->row_step + col_start_ * msg->point_step];
            uint8_t* dst_ptr = &cropped.data[row * cropped.row_step];
            std::memcpy(dst_ptr, src_ptr, cropped.row_step);
        }
        
        pub_pointcloud_.publish(cropped);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "pointcloud_crop_node");
    ROS_INFO("[pointcloud_crop_node] Starting PointCloud Crop Node");
    PointCloudCropNode node;
    ros::spin();
    return 0;
}
