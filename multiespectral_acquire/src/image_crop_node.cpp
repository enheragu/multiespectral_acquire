/**
 * @file    image_crop_node.cpp
 * @brief   Generic image cropping node - crops any image topic by FOV and republishes with _cropped suffix
 */

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "core/utils/logging_utils.h"
#include "ros_utils/ros_logger.h"

class ImageCropNode {
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    std::shared_ptr<Logger> logger_;
    
    image_transport::ImageTransport it_;
    image_transport::Subscriber sub_image_;
    image_transport::Publisher pub_image_;
    
    std::string input_topic_;
    std::string output_topic_;
    
    bool fov_enabled_;
    bool use_angular_;
    
    // Pixel-based FOV
    int col_start_, col_end_;
    int row_start_, row_end_;
    
    // Angular FOV (converted to pixels on first image)
    double h_fov_min_deg_, h_fov_max_deg_;
    double v_fov_min_deg_, v_fov_max_deg_;
    bool fov_computed_;
    
public:
    ImageCropNode() : nh_(), pnh_("~"), it_(nh_), fov_computed_(false) {
        logger_ = std::make_shared<RosLogger>();
        
        // Parameters
        pnh_.param<std::string>("input_topic", input_topic_, "");
        
        if (input_topic_.empty()) {
            logger_->error_stream() << "[ImageCropNode] ERROR: input_topic parameter is required!";
            ros::shutdown();
            return;
        }
        
        // Output topic is always input + "_cropped"
        output_topic_ = input_topic_ + "_cropped";
        
        loadFOVConfig();
        
        // Publishers/Subscribers
        pub_image_ = it_.advertise(output_topic_, 1);
        sub_image_ = it_.subscribe(input_topic_, 10, &ImageCropNode::image_cb, this);
        
        logger_->info_stream() << "[ImageCropNode] Initialized. Input: " << input_topic_
                               << " -> Output: " << output_topic_
                               << ", FOV enabled: " << (fov_enabled_ ? "YES" : "NO");
    }
    
    void loadFOVConfig() {
        pnh_.param<bool>("fov_enabled", fov_enabled_, false);
        if (!fov_enabled_) return;
        
        pnh_.param<bool>("fov_use_angular", use_angular_, false);
        if (use_angular_) {
            pnh_.param<double>("fov_h_min_deg", h_fov_min_deg_, -45.0);
            pnh_.param<double>("fov_h_max_deg", h_fov_max_deg_, 45.0);
            pnh_.param<double>("fov_v_min_deg", v_fov_min_deg_, -22.5);
            pnh_.param<double>("fov_v_max_deg", v_fov_max_deg_, 22.5);
            logger_->info_stream() << "[ImageCropNode] Angular FOV: H[" << h_fov_min_deg_ 
                                   << "," << h_fov_max_deg_ << "], V[" 
                                   << v_fov_min_deg_ << "," << v_fov_max_deg_ << "]";
        } else {
            pnh_.param<int>("fov_col_start", col_start_, -1);
            pnh_.param<int>("fov_col_end", col_end_, -1);
            pnh_.param<int>("fov_row_start", row_start_, -1);
            pnh_.param<int>("fov_row_end", row_end_, -1);
            logger_->info_stream() << "[ImageCropNode] Pixel FOV: cols[" << col_start_ 
                                   << "," << col_end_ << "], rows[" 
                                   << row_start_ << "," << row_end_ << "]";
        }
    }
    
    void computePixelFOV(int width, int height) {
        if (fov_computed_) return;
        
        // Assume horizontal FOV of 360° and vertical FOV based on Ouster specs
        double total_h_fov = 360.0;
        double total_v_fov = 45.0; // Typical for OS1-64
        
        // Convert angular FOV to column/row indices
        col_start_ = static_cast<int>((h_fov_min_deg_ + total_h_fov / 2.0) / total_h_fov * width);
        col_end_ = static_cast<int>((h_fov_max_deg_ + total_h_fov / 2.0) / total_h_fov * width);
        
        row_start_ = static_cast<int>((total_v_fov / 2.0 - v_fov_max_deg_) / total_v_fov * height);
        row_end_ = static_cast<int>((total_v_fov / 2.0 - v_fov_min_deg_) / total_v_fov * height);
        
        // Clamp to valid ranges
        col_start_ = std::max(0, std::min(col_start_, width - 1));
        col_end_ = std::max(0, std::min(col_end_, width - 1));
        row_start_ = std::max(0, std::min(row_start_, height - 1));
        row_end_ = std::max(0, std::min(row_end_, height - 1));
        
        logger_->info_stream() << "[ImageCropNode] Computed pixel FOV from angular: cols[" 
                               << col_start_ << "," << col_end_ << "], rows[" 
                               << row_start_ << "," << row_end_ << "]";
        
        fov_computed_ = true;
    }
    
    void image_cb(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg);
        } catch (cv_bridge::Exception& e) {
            logger_->error_stream() << "[ImageCropNode] cv_bridge exception: " << e.what();
            return;
        }
        
        cv::Mat cropped_image;
        
        if (fov_enabled_) {
            // Compute pixel FOV from angular on first image
            if (use_angular_ && !fov_computed_) {
                computePixelFOV(cv_ptr->image.cols, cv_ptr->image.rows);
            }
            
            // Validate ranges
            int width = cv_ptr->image.cols;
            int height = cv_ptr->image.rows;
            
            if (col_start_ >= 0 && col_end_ > col_start_ && col_end_ < width &&
                row_start_ >= 0 && row_end_ > row_start_ && row_end_ < height) {
                
                // Crop the image
                cv::Rect roi(col_start_, row_start_, 
                            col_end_ - col_start_, 
                            row_end_ - row_start_);
                cropped_image = cv_ptr->image(roi).clone();
            } else {
                // Invalid FOV, publish original
                cropped_image = cv_ptr->image;
            }
        } else {
            // No crop, publish original
            cropped_image = cv_ptr->image;
        }
        
        // Publish cropped image
        if (pub_image_.getNumSubscribers() > 0) {
            cv_ptr->image = cropped_image;
            pub_image_.publish(cv_ptr->toImageMsg());
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_crop_node");
    ROS_INFO("[image_crop_node] Starting Image Crop Node");
    ImageCropNode node;
    ros::spin();
    return 0;
}
