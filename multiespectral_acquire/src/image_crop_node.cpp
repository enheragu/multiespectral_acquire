/**
 * @file    image_crop_node.cpp
 * @brief   Generic image cropping node - crops any image topic by FOV and republishes with _cropped suffix
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include "core/utils/logging_utils.h"
#include "ros_utils/ros_logger.h"

class ImageCropNode : public rclcpp::Node {
    std::shared_ptr<Logger> logger_;

    image_transport::Publisher  pub_image_;
    image_transport::Subscriber sub_image_;

    std::string input_topic_;
    std::string output_topic_;

    bool fov_enabled_;
    bool use_angular_;

    int col_start_, col_end_;
    int row_start_, row_end_;

    double h_fov_min_deg_, h_fov_max_deg_;
    double v_fov_min_deg_, v_fov_max_deg_;
    bool fov_computed_ = false;

public:
    ImageCropNode() : rclcpp::Node("image_crop_node") {
        logger_ = std::make_shared<RosLogger>(this->get_logger());

        this->declare_parameter<std::string>("input_topic", "");
        this->declare_parameter<bool>("fov_enabled", false);
        this->declare_parameter<bool>("fov_use_angular", false);
        this->declare_parameter<double>("fov_h_min_deg", -45.0);
        this->declare_parameter<double>("fov_h_max_deg",  45.0);
        this->declare_parameter<double>("fov_v_min_deg", -22.5);
        this->declare_parameter<double>("fov_v_max_deg",  22.5);
        this->declare_parameter<double>("sensor_v_fov_deg", 90.0);
        this->declare_parameter<int>("fov_col_start", -1);
        this->declare_parameter<int>("fov_col_end",   -1);
        this->declare_parameter<int>("fov_row_start", -1);
        this->declare_parameter<int>("fov_row_end",   -1);

        input_topic_ = this->get_parameter("input_topic").as_string();
        if (input_topic_.empty()) {
            logger_->error_stream() << "[ImageCropNode] ERROR: input_topic parameter is required!";
            rclcpp::shutdown();
            return;
        }

        output_topic_ = input_topic_ + "_cropped";
        loadFOVConfig();

        pub_image_ = image_transport::create_publisher(this, output_topic_);
        sub_image_ = image_transport::create_subscription(
            this, input_topic_,
            std::bind(&ImageCropNode::image_cb, this, std::placeholders::_1),
            "raw");

        logger_->info_stream() << "[ImageCropNode] Initialized. Input: " << input_topic_
                               << " -> Output: " << output_topic_
                               << ", FOV enabled: " << (fov_enabled_ ? "YES" : "NO");
    }

private:
    void loadFOVConfig() {
        fov_enabled_ = this->get_parameter("fov_enabled").as_bool();
        if (!fov_enabled_) return;

        use_angular_ = this->get_parameter("fov_use_angular").as_bool();
        if (use_angular_) {
            h_fov_min_deg_ = this->get_parameter("fov_h_min_deg").as_double();
            h_fov_max_deg_ = this->get_parameter("fov_h_max_deg").as_double();
            v_fov_min_deg_ = this->get_parameter("fov_v_min_deg").as_double();
            v_fov_max_deg_ = this->get_parameter("fov_v_max_deg").as_double();
            logger_->info_stream() << "[ImageCropNode] Angular FOV: H["
                << h_fov_min_deg_ << "," << h_fov_max_deg_ << "], V["
                << v_fov_min_deg_ << "," << v_fov_max_deg_ << "]";
        } else {
            col_start_ = this->get_parameter("fov_col_start").as_int();
            col_end_   = this->get_parameter("fov_col_end").as_int();
            row_start_ = this->get_parameter("fov_row_start").as_int();
            row_end_   = this->get_parameter("fov_row_end").as_int();
            logger_->info_stream() << "[ImageCropNode] Pixel FOV: cols["
                << col_start_ << "," << col_end_ << "], rows["
                << row_start_ << "," << row_end_ << "]";
        }
    }

    void computePixelFOV(int width, int height) {
        if (fov_computed_) return;

        constexpr double total_h_fov = 360.0;
        const double total_v_fov = this->get_parameter("sensor_v_fov_deg").as_double();

        col_start_ = static_cast<int>((h_fov_min_deg_ + total_h_fov / 2.0) / total_h_fov * width);
        col_end_   = static_cast<int>((h_fov_max_deg_ + total_h_fov / 2.0) / total_h_fov * width);
        row_start_ = static_cast<int>((total_v_fov / 2.0 - v_fov_max_deg_) / total_v_fov * height);
        row_end_   = static_cast<int>((total_v_fov / 2.0 - v_fov_min_deg_) / total_v_fov * height);

        col_start_ = std::max(0, std::min(col_start_, width  - 1));
        col_end_   = std::max(0, std::min(col_end_,   width  - 1));
        row_start_ = std::max(0, std::min(row_start_, height - 1));
        row_end_   = std::max(0, std::min(row_end_,   height - 1));

        logger_->info_stream() << "[ImageCropNode] Computed pixel FOV from angular: cols["
            << col_start_ << "," << col_end_ << "], rows["
            << row_start_ << "," << row_end_ << "]";
        fov_computed_ = true;
    }

    void image_cb(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg);
        } catch (const cv_bridge::Exception& e) {
            logger_->error_stream() << "[ImageCropNode] cv_bridge exception: " << e.what();
            return;
        }

        cv::Mat result;

        if (fov_enabled_) {
            if (use_angular_ && !fov_computed_) {
                computePixelFOV(cv_ptr->image.cols, cv_ptr->image.rows);
            }

            int w = cv_ptr->image.cols;
            int h = cv_ptr->image.rows;

            if (col_start_ >= 0 && col_end_ > col_start_ && col_end_ < w &&
                row_start_ >= 0 && row_end_ > row_start_ && row_end_ < h)
            {
                cv::Rect roi(col_start_, row_start_,
                             col_end_ - col_start_,
                             row_end_ - row_start_);
                result = cv_ptr->image(roi).clone();
            } else {
                result = cv_ptr->image;
            }
        } else {
            result = cv_ptr->image;
        }

        if (pub_image_.getNumSubscribers() > 0) {
            cv_ptr->image = result;
            pub_image_.publish(cv_ptr->toImageMsg());
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("image_crop_node"),
                "[image_crop_node] Starting Image Crop Node");
    rclcpp::spin(std::make_shared<ImageCropNode>());
    rclcpp::shutdown();
    return 0;
}
