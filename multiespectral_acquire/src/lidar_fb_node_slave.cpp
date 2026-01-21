#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "multiespectral_acquire/ImageRequest.h"
#include "lidar_drivers/lidar_adapter.h"

int LIDAR_FRAME_RATE = 20; // Real frame rate of the sensor

inline PointCloud2Data toPointCloud2Data(const sensor_msgs::PointCloud2& ros_cloud) {
    PointCloud2Data cloud;
    cloud.height = ros_cloud.height;
    cloud.width = ros_cloud.width;
    cloud.point_step = ros_cloud.point_step;
    cloud.row_step = ros_cloud.row_step;
    cloud.is_dense = ros_cloud.is_dense;
    cloud.is_bigendian = ros_cloud.is_bigendian;
    cloud.timestamp_ns = static_cast<uint64_t>(ros_cloud.header.stamp.sec) * 1000000000ULL 
                       + static_cast<uint64_t>(ros_cloud.header.stamp.nsec);
    cloud.seq = ros_cloud.header.seq;
    cloud.frame_id = ros_cloud.header.frame_id;
    cloud.data = ros_cloud.data;
    
    cloud.fields.reserve(ros_cloud.fields.size());
    for (const auto& f : ros_cloud.fields) {
        PointCloud2Data::FieldInfo field;
        field.name = f.name;
        field.offset = f.offset;
        field.datatype = f.datatype;
        field.count = f.count;
        cloud.fields.push_back(field);
    }
    return cloud;
}

inline sensor_msgs::PointCloud2 toRosPointCloud2(const PointCloud2Data& cloud, const std::string& frame_id = "lidar") {
    sensor_msgs::PointCloud2 ros_cloud;
    ros_cloud.height = cloud.height;
    ros_cloud.width = cloud.width;
    ros_cloud.point_step = cloud.point_step;
    ros_cloud.row_step = cloud.row_step;
    ros_cloud.is_dense = cloud.is_dense;
    ros_cloud.is_bigendian = cloud.is_bigendian;
    ros_cloud.header.stamp.sec = cloud.timestamp_ns / 1000000000ULL;
    ros_cloud.header.stamp.nsec = cloud.timestamp_ns % 1000000000ULL;
    ros_cloud.header.seq = cloud.seq;
    ros_cloud.header.frame_id = frame_id;
    ros_cloud.data = cloud.data;
    
    ros_cloud.fields.reserve(cloud.fields.size());
    for (const auto& f : cloud.fields) {
        sensor_msgs::PointField field;
        field.name = f.name;
        field.offset = f.offset;
        field.datatype = f.datatype;
        field.count = f.count;
        ros_cloud.fields.push_back(field);
    }
    return ros_cloud;
}

class LidarAcquire : public LidarAdapterSlave {
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::ServiceServer service_;
    
    ros::Publisher pc_pub_;
    ros::Publisher img_pub_;
    
    std::string pc_topic_;
    std::string img_topic_;
    std::string frame_id_;

public:
    LidarAcquire(const std::string& topic, const std::string& dataset_output_path) {
        setLogger(std::make_shared<RosLogger>("LidarAcquire"));
        
        int configured_rate = 10;
        nh_.param<int>("frame_rate", configured_rate, 10);
        nh_.param<int>("lidar_acq_frame_rate", LIDAR_FRAME_RATE, LIDAR_FRAME_RATE);
        nh_.param<std::string>("pointcloud_topic", pc_topic_, "lidar_pointcloud");
        nh_.param<std::string>("image_topic", img_topic_, "lidar_intensity");
        nh_.param<std::string>("frame_id", frame_id_, "lidar");
        
        init_buffer(LIDAR_FRAME_RATE, configured_rate);
        init_store_folder(dataset_output_path);
        loadFOVConfig();
        
        sub_ = nh_.subscribe(topic, 10, &LidarAcquire::pointcloud_cb, this);
        service_ = nh_.advertiseService("lidar_slave_service", &LidarAcquire::service_cb, this);
        pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(pc_topic_, 1);
        img_pub_ = nh_.advertise<sensor_msgs::Image>(img_topic_, 1);
        
        logger_->info_stream() << "[LADriver] Initialized. Input: " << topic 
                               << ", PC out: " << pc_topic_ << ", Img out: " << img_topic_;
    }
    
    void loadFOVConfig() {
        nh_.param<bool>("fov_enabled", fov_enabled_, false);
        if (!fov_enabled_) return;
        
        nh_.param<bool>("fov_use_angular", fov_config_.use_angular_fov, false);
        if (fov_config_.use_angular_fov) {
            nh_.param<double>("fov_h_min_deg", fov_config_.h_fov_min_deg, -45.0);
            nh_.param<double>("fov_h_max_deg", fov_config_.h_fov_max_deg, 45.0);
            nh_.param<double>("fov_v_min_deg", fov_config_.v_fov_min_deg, -22.5);
            nh_.param<double>("fov_v_max_deg", fov_config_.v_fov_max_deg, 22.5);
        } else {
            nh_.param<int>("fov_col_start", fov_config_.col_start, -1);
            nh_.param<int>("fov_col_end", fov_config_.col_end, -1);
            nh_.param<int>("fov_row_start", fov_config_.row_start, -1);
            nh_.param<int>("fov_row_end", fov_config_.row_end, -1);
        }
    }
    
    void pointcloud_cb(const sensor_msgs::PointCloud2ConstPtr& msg) {
        PointCloud2Data cloud = toPointCloud2Data(*msg);
        
        LidarFrameData frame;
        processPointCloud(cloud, frame, fov_config_, fov_enabled_);
        frame.fillMetadata(cloud.timestamp_ns, cloud.seq, dataset_name_, LIDAR_FRAME_RATE);
        addFrameToBuffer(frame);
        
        // Publish processed pointcloud
        if (pc_pub_.getNumSubscribers() > 0) {
            sensor_msgs::PointCloud2 pc_msg = toRosPointCloud2(frame.pointcloud, frame_id_);
            pc_pub_.publish(pc_msg);
        }
        
        // Publish intensity image
        if (img_pub_.getNumSubscribers() > 0 && !frame.intensity_image.empty()) {
            std_msgs::Header header;
            header.stamp.sec = cloud.timestamp_ns / 1000000000ULL;
            header.stamp.nsec = cloud.timestamp_ns % 1000000000ULL;
            header.frame_id = frame_id_;
            cv_bridge::CvImage cv_img(header, "mono8", frame.intensity_image);
            img_pub_.publish(cv_img.toImageMsg());
        }
    }

    bool service_cb(multiespectral_acquire::ImageRequest::Request &request,
                    multiespectral_acquire::ImageRequest::Response &response) {
        if (buffer_.empty()) {
            logger_->warn_stream() << "[LADriver::service_cb] Buffer is empty.";
            response.success = false;
            return false;
        }
        
        uint64_t timestamp = request.timestamp;
        auto closest_it = std::min_element(buffer_.begin(), buffer_.end(),
            [timestamp](const LidarFrameData& a, const LidarFrameData& b) {
                return std::abs(static_cast<int64_t>(a.timestamp - timestamp)) <
                       std::abs(static_cast<int64_t>(b.timestamp - timestamp));
            });
            
        if (closest_it == buffer_.end()) {
            response.success = false;
            return false;
        }
        
        double time_diff_s = std::abs(static_cast<int64_t>(closest_it->timestamp - timestamp)) / 1e9;
        double max_diff = 1.0 / double(std::max(1, LIDAR_FRAME_RATE - 1));
        if (time_diff_s > max_diff) {
            logger_->warn_stream() << "[LADriver::service_cb] Time diff " << time_diff_s << "s > max " << max_diff << "s";
            response.success = false;
            return true;
        }
        
        closest_it->setReferencePair(request.reference_pair);
        
        if (request.store) {
            closest_it->save(pc_path_, intensity_path_, request.reference_pair);
        }
        
        response.success = true;
        return true;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_fb_node_slave");
    ros::NodeHandle nh("~");
    
    std::string topic, dataset_output_path;
    nh.param<std::string>("input_lidar_topic", topic, "/os_cloud_node/points");
    nh.param<std::string>("dataset_output_path", dataset_output_path, ".");
    
    LidarAcquire node(topic, dataset_output_path);
    ros::spin();
    return 0;
}