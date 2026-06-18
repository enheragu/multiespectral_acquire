/**
 * @file    camera_handler_node.cpp
 * @brief   Simplified camera node that only acquires and publishes images with metadata
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include "multiespectral_acquire/msg/image_with_metadata.hpp"
#include "multiespectral_acquire/msg/image_metadata.hpp"
#include "core/camera_drivers/camera_adapter.h"
#include "core/utils/logging_utils.h"
#include "core/utils/image_metadata.h"
#include "ros_utils/ros_logger.h"

static void fillImageMetadataMsg(const ImageMetadata& meta,
                                 multiespectral_acquire::msg::ImageMetadata& msg)
{
    msg.camera_timestamp       = meta.camera_timestamp;
    msg.exposure_time          = meta.exposureTime;
    msg.half_exposure_timestamp = meta.half_exposure_timestamp;
    msg.frame_counter          = meta.frameCounter;
    msg.gain                   = meta.gain;
    msg.width                  = meta.width;
    msg.height                 = meta.height;
    msg.pixel_format           = meta.pixelFormat;
    msg.timetag                = meta.timetag;
    msg.img_name               = meta.img_name;
    msg.img_pair_name          = meta.img_pair_name;
    msg.dataset_name           = meta.dataset_name;
}

class CameraHandlerNode : public rclcpp::Node, public CameraAdapter {
    rclcpp::Publisher<multiespectral_acquire::msg::ImageWithMetadata>::SharedPtr pub_image_metadata_;
    image_transport::Publisher it_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_pub_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;

    rclcpp::TimerBase::SharedPtr timer_;
    std::string topic_name_;
    std::string camera_info_cfg_;

    int consecutive_errors_ = 0;
    static constexpr int MAX_CONSECUTIVE_ERRORS = 10;

public:
    CameraHandlerNode() : rclcpp::Node("camera_handler_node") {
        this->setLogger(std::make_shared<RosLogger>(this->get_logger()));

        // Parameters
        this->declare_parameter<std::string>("image_topic", getType() + "_image");
        this->declare_parameter<double>("output_frame_rate", 1.0);
        this->declare_parameter<std::string>("camera_ip", "");
        this->declare_parameter<std::string>("camera_info_url", "");
        this->declare_parameter<bool>("use_ptp", true);
        this->declare_parameter<std::string>("dataset_name", getFolderTimetag());

        topic_name_      = this->get_parameter("image_topic").as_string();
        frame_rate       = this->get_parameter("output_frame_rate").as_double();
        camera_ip        = this->get_parameter("camera_ip").as_string();
        camera_info_cfg_ = this->get_parameter("camera_info_url").as_string();
        force_disable_ptp = !this->get_parameter("use_ptp").as_bool();
        dataset_name     = this->get_parameter("dataset_name").as_string();

        setNodeName(this->get_fully_qualified_name());

        logger_->info_stream() << "[CameraHandlerNode] Configured: topic=" << topic_name_
                               << ", rate=" << frame_rate << "Hz, session=" << dataset_name;

        // Publishers (image_transport free functions work with raw Node* in constructor)
        pub_image_metadata_ = this->create_publisher<multiespectral_acquire::msg::ImageWithMetadata>(
            topic_name_ + "/image_with_metadata", 1);
        it_pub_ = image_transport::create_publisher(this, topic_name_);
        cam_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            topic_name_ + "/camera_info", 1);

        // Initialize camera
        if (!this->init(static_cast<int>(frame_rate))) {
            logger_->fatal_stream() << "[CameraHandlerNode] Failed to initialize camera";
            rclcpp::shutdown();
            return;
        }

        // Camera info manager
        if (!camera_info_cfg_.empty()) {
            cinfo_ = std::make_shared<camera_info_manager::CameraInfoManager>(
                this, getModelName(), camera_info_cfg_);
        }

        // Acquisition timer
        auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / frame_rate));
        timer_ = this->create_wall_timer(period_ns,
            std::bind(&CameraHandlerNode::acquisition_cb, this));

        logger_->info_stream() << "[CameraHandlerNode] " << getName()
                               << " initialized at IP=" << camera_ip
                               << " rate=" << frame_rate << "Hz";
    }

    void acquisition_cb() {
        cv::Mat curr_image(480, 640, CV_8UC1, cv::Scalar(0));
        createTestPattern(curr_image);

        ImageMetadata metadata;
        metadata.dataset_name = dataset_name;
        metadata.setROSTimeNowCallback([this]() {
            return static_cast<uint64_t>(this->now().nanoseconds());
        });

        if (!grabImage(curr_image, metadata) || curr_image.empty()) {
            ++consecutive_errors_;
            logger_->error_stream() << "[CameraHandlerNode] Failed to grab image ("
                                    << consecutive_errors_ << "/" << MAX_CONSECUTIVE_ERRORS << ")";
            if (consecutive_errors_ >= MAX_CONSECUTIVE_ERRORS) {
                logger_->fatal_stream() << "[CameraHandlerNode] Too many consecutive errors, shutting down";
                rclcpp::shutdown();
            }
            return;
        }
        consecutive_errors_ = 0;

        std::string encoding;
        if      (curr_image.type() == CV_8UC3) { encoding = "bgr8";  }
        else if (curr_image.type() == CV_8UC1) { encoding = "mono8"; }
        else {
            logger_->error_stream() << "Unsupported image type: " << curr_image.type();
            return;
        }

        std_msgs::msg::Header header;
        header.stamp.sec     = static_cast<int32_t>(metadata.camera_timestamp / 1000000000ULL);
        header.stamp.nanosec = static_cast<uint32_t>(metadata.camera_timestamp % 1000000000ULL);
        header.frame_id      = getName() + "_frame";

        auto img_msg = cv_bridge::CvImage(header, encoding, curr_image).toImageMsg();

        // Secondary publishers only pay serialization when someone listens.
        // At 30 Hz (FLIR) the unconditional duplicate publish of every frame
        // doubled the DDS payload for nothing.
        if (it_pub_.getNumSubscribers() > 0) {
            it_pub_.publish(*img_msg);
        }

        if (cinfo_ && cam_info_pub_->get_subscription_count() > 0) {
            auto cam_info = cinfo_->getCameraInfo();
            cam_info.header = header;
            cam_info_pub_->publish(cam_info);
        }

        multiespectral_acquire::msg::ImageWithMetadata msg;
        // it_pub_.publish(*img_msg) serializes synchronously, so the image can
        // be moved instead of copied (saves a full-frame memcpy per frame).
        msg.image = std::move(*img_msg);
        fillImageMetadataMsg(metadata, msg.metadata);
        msg.metadata.header = header;
        pub_image_metadata_->publish(msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("camera_handler_node"),
                "[camera_handler_node] Starting Camera Handler Node");
    int rc = 0;
    try {
        rclcpp::spin(std::make_shared<CameraHandlerNode>());
    } catch (const std::exception& e) {
        // CameraAdapter::init throws on camera init/calibration failure.
        // Exit nonzero so launch respawn restarts us, without the SIGABRT
        // core-dump noise of an uncaught exception.
        RCLCPP_FATAL(rclcpp::get_logger("camera_handler_node"),
                     "[camera_handler_node] Fatal: %s", e.what());
        rc = 1;
    }
    rclcpp::shutdown();
    return rc;
}
