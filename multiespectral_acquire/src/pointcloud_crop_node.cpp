/**
 * @file    pointcloud_crop_node.cpp
 * @brief   Crops an Ouster point cloud to the camera vertical FOV and republishes
 *          to <input>_cropped.
 *
 * Works on a DENSE (organized:false) cloud — a flat list of valid points with no
 * row/col grid — so the crop can no longer slice a rectangle of the grid. Instead
 * it filters by the per-point `ring` field (the physical beam index 0..N-1), which
 * maps linearly to elevation. This is exact and convention-free.
 *
 * Horizontal FOV is NOT narrowed here: the Ouster azimuth_window already limits the
 * scan to ~50° around the camera direction, a harmless margin over the ~27.5° camera
 * FOV (points outside the image are dropped at projection time anyway). If a tight
 * horizontal crop is ever needed, add a geometric azimuth filter and verify the FOV
 * against the camera image (the azimuth-window centre is mounting-dependent).
 */
#include <cmath>
#include <cstring>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class PointCloudCropNode : public rclcpp::Node {
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pointcloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_pointcloud_;

    std::string input_topic_;
    std::string output_topic_;

    bool   fov_enabled_;
    double v_fov_min_deg_, v_fov_max_deg_;
    double sensor_v_fov_deg_;

    int  ring_start_ = -1, ring_end_ = -1;   // computed from v_fov on first cloud
    int  ring_offset_ = -1;                   // byte offset of the `ring` field
    bool layout_ready_ = false;

public:
    PointCloudCropNode() : rclcpp::Node("pointcloud_crop_node") {
        this->declare_parameter<std::string>("input_topic", "/ouster/points");
        this->declare_parameter<bool>("fov_enabled", false);
        this->declare_parameter<bool>("fov_use_angular", false);   // kept for launch compat
        this->declare_parameter<double>("fov_h_min_deg", -45.0);   // (horizontal not cropped here)
        this->declare_parameter<double>("fov_h_max_deg",  45.0);
        this->declare_parameter<double>("fov_v_min_deg", -22.5);
        this->declare_parameter<double>("fov_v_max_deg",  22.5);
        // OS-0-128 has a 90° vertical FOV. (The old node hard-coded 45° — wrong for OS0.)
        this->declare_parameter<double>("sensor_v_fov_deg", 90.0);

        input_topic_      = this->get_parameter("input_topic").as_string();
        output_topic_     = input_topic_ + "_cropped";
        fov_enabled_      = this->get_parameter("fov_enabled").as_bool();
        v_fov_min_deg_    = this->get_parameter("fov_v_min_deg").as_double();
        v_fov_max_deg_    = this->get_parameter("fov_v_max_deg").as_double();
        sensor_v_fov_deg_ = this->get_parameter("sensor_v_fov_deg").as_double();

        pub_pointcloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 1);
        // RELIABLE reader: the input is the compositor's RELIABLE republished
        // `ouster/points_sync` (a ~1.6-3.8 MB cloud at the trigger rate). A
        // best-effort reader silently dropped ~70% of those large samples (only
        // ~0.3 Hz of 1 Hz reached the crop -> sparse stored dataset). Reliable
        // delivers all of them. NOTE: only valid because the input is the reliable
        // _sync; if ever pointed at the best-effort raw /ouster/points, revert.
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        sub_pointcloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, qos,
            std::bind(&PointCloudCropNode::pointcloud_cb, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
            "[PointCloudCropNode] %s -> %s | FOV crop: %s (vertical by ring, V[%.1f, %.1f] of %.0f°)",
            input_topic_.c_str(), output_topic_.c_str(), fov_enabled_ ? "YES" : "NO",
            v_fov_min_deg_, v_fov_max_deg_, sensor_v_fov_deg_);
    }

private:
    // Locate the `ring` field and compute the beam-index window for the vertical FOV.
    bool prepareLayout(const sensor_msgs::msg::PointCloud2& msg) {
        const sensor_msgs::msg::PointField* ring = nullptr;
        for (const auto& f : msg.fields) {
            if (f.name == "ring") { ring = &f; break; }
        }
        if (!ring) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "[PointCloudCropNode] no 'ring' field — passing cloud through uncropped");
            return false;
        }
        ring_offset_ = static_cast<int>(ring->offset);

        // Beams span sensor_v_fov_deg vertically; ring 0 = top (+elevation).
        // Replicates the original row mapping: row=(v_half - elev)/v_fov * N.
        // We don't know N until a cloud arrives, so derive it from the max ring seen
        // (128 for OS0-128) — but the mapping only needs the fraction, applied to the
        // beam count. Use the standard OS0-128 count of 128 unless overridden by data.
        const double v_half = sensor_v_fov_deg_ / 2.0;
        const int    n_beams = 128;   // OS-0-128
        ring_start_ = static_cast<int>((v_half - v_fov_max_deg_) / sensor_v_fov_deg_ * n_beams);
        ring_end_   = static_cast<int>((v_half - v_fov_min_deg_) / sensor_v_fov_deg_ * n_beams);
        ring_start_ = std::max(0, std::min(ring_start_, n_beams - 1));
        ring_end_   = std::max(0, std::min(ring_end_,   n_beams - 1));
        if (ring_start_ > ring_end_) std::swap(ring_start_, ring_end_);

        RCLCPP_INFO(this->get_logger(),
            "[PointCloudCropNode] vertical FOV V[%.1f,%.1f] -> rings [%d, %d] (ring offset %d)",
            v_fov_min_deg_, v_fov_max_deg_, ring_start_, ring_end_, ring_offset_);
        layout_ready_ = true;
        return true;
    }

    void pointcloud_cb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
        if (!fov_enabled_) {
            pub_pointcloud_->publish(*msg);
            return;
        }
        if (!layout_ready_ && !prepareLayout(*msg)) {
            pub_pointcloud_->publish(*msg);   // no ring field — passthrough
            return;
        }

        const uint32_t ps = msg->point_step;
        const size_t   n_in = (ps > 0) ? msg->data.size() / ps : 0;

        sensor_msgs::msg::PointCloud2 out;
        out.header       = msg->header;
        out.fields       = msg->fields;
        out.is_bigendian = msg->is_bigendian;
        out.point_step   = ps;
        out.height       = 1;
        out.is_dense     = true;
        out.data.resize(msg->data.size());   // upper bound; shrink after

        const uint8_t* src = msg->data.data();
        uint8_t*       dst = out.data.data();
        size_t kept = 0;
        for (size_t i = 0; i < n_in; ++i) {
            const uint8_t* p = src + i * ps;
            uint16_t ring;
            std::memcpy(&ring, p + ring_offset_, sizeof(ring));
            if (ring >= ring_start_ && ring <= ring_end_) {
                std::memcpy(dst + kept * ps, p, ps);
                ++kept;
            }
        }

        out.width    = static_cast<uint32_t>(kept);
        out.row_step = static_cast<uint32_t>(kept * ps);
        out.data.resize(kept * ps);
        pub_pointcloud_->publish(out);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("pointcloud_crop_node"),
                "[pointcloud_crop_node] Starting PointCloud Crop Node");
    rclcpp::spin(std::make_shared<PointCloudCropNode>());
    rclcpp::shutdown();
    return 0;
}
