#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "multiespectral_acquire/ImageRequest.h"
#include "lidar_drivers/lidar_adapter.h"
#include "camera_adapter_ros.h"
#include "utils/timed_frame_buffer.h"

int LIDAR_FRAME_RATE = 20.0; // Real frame rate of the sensor

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
    ros::NodeHandle pnh_;
    ros::Subscriber sub_;
    ros::ServiceServer service_;

    ros::Publisher pc_pub_;
    image_transport::ImageTransport it_;
    image_transport::Publisher it_img_pub_;
    
    std::string pc_topic_;
    std::string img_topic_;
    std::string frame_id_;

    ros::Subscriber gnss_data_sub_;
    ros::Subscriber odom_data_sub_;

    TimedFrameBuffer<GNSSDataBuffer> gnss_data_buffer_;
    TimedFrameBuffer<OdomDataBuffer> odom_data_buffer_;

public:
    LidarAcquire(const std::string& topic, const std::string& dataset_output_path)
        : nh_(), pnh_("~"), it_(nh_) {
        this->setLogger(std::make_shared<RosLogger>());
        
        int configured_rate = 10;
        pnh_.param<int>("frame_rate", configured_rate, 10);
        pnh_.param<int>("lidar_acq_frame_rate", LIDAR_FRAME_RATE, LIDAR_FRAME_RATE);
        pnh_.param<std::string>("pointcloud_topic", pc_topic_, "lidar_pointcloud");
        pnh_.param<std::string>("image_topic", img_topic_, "swir_lidar");
        pnh_.param<std::string>("frame_id", frame_id_, "lidar");
        
        init_store_folder(dataset_output_path);
        loadFOVConfig();
        
        sub_ = nh_.subscribe(topic, 10, &LidarAcquire::pointcloud_cb, this);
        service_ = nh_.advertiseService("lidar_slave_service", &LidarAcquire::service_cb, this);
        pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(pc_topic_, 1);
        it_img_pub_ = it_.advertise(img_topic_, 1);
        
        logger_->info_stream() << SUCCEED_F << "[LADriver] Initialized. Input: " << topic 
                               << ", PC out: " << pc_topic_ << ", Img out: " << img_topic_ << RESET_F;


        std::string gnss_topic_name, odom_topic_name;
        pnh_.param<std::string>("gnss_topic", gnss_topic_name, "-");
        pnh_.param<std::string>("odom_topic", odom_topic_name, "-");
        if (gnss_topic_name != "-")
        {
            logger_->info_stream() << "[LADriver] Subscribing to GNSS topic: " << gnss_topic_name;
            gnss_data_sub_ = nh_.subscribe(gnss_topic_name, 10, &LidarAcquire::gnss_data_cb, this);
        }
        if (odom_topic_name != "-") 
        {
            logger_->info_stream() << "[LADriver] Subscribing to odom topic: " << odom_topic_name;
            odom_data_sub_ = nh_.subscribe(odom_topic_name, 10, &LidarAcquire::odom_data_cb, this);
        }
    }
    
    void loadFOVConfig() {
        pnh_.param<bool>("fov_enabled", fov_enabled_, false);
        if (!fov_enabled_) return;

        pnh_.param<bool>("fov_use_angular", fov_config_.use_angular_fov, false);
        if (fov_config_.use_angular_fov) {
            pnh_.param<double>("fov_h_min_deg", fov_config_.h_fov_min_deg, -45.0);
            pnh_.param<double>("fov_h_max_deg", fov_config_.h_fov_max_deg, 45.0);
            pnh_.param<double>("fov_v_min_deg", fov_config_.v_fov_min_deg, -22.5);
            pnh_.param<double>("fov_v_max_deg", fov_config_.v_fov_max_deg, 22.5);
        } else {
            pnh_.param<int>("fov_col_start", fov_config_.col_start, -1);
            pnh_.param<int>("fov_col_end", fov_config_.col_end, -1);
            pnh_.param<int>("fov_row_start", fov_config_.row_start, -1);
            pnh_.param<int>("fov_row_end", fov_config_.row_end, -1);
        }
    }
    
    void pointcloud_cb(const sensor_msgs::PointCloud2ConstPtr& msg) {
        PointCloud2Data cloud = toPointCloud2Data(*msg);
        
        LidarFrameData frame;
        processPointCloud(cloud, frame, fov_config_, fov_enabled_);
        
        frame.fillMetadata(cloud.timestamp_ns, cloud.seq, dataset_name_, LIDAR_FRAME_RATE);
        buffer_.addFrame(frame);
        
        // Publish processed pointcloud
        if (pc_pub_.getNumSubscribers() > 0) {
            sensor_msgs::PointCloud2 pc_msg = toRosPointCloud2(frame.pointcloud, frame_id_);
            pc_pub_.publish(pc_msg);
        }
        
        // Publish intensity image
        if (it_img_pub_.getNumSubscribers() > 0 && !frame.intensity_image.empty()) {
            std_msgs::Header header;
            header.stamp.sec = cloud.timestamp_ns / 1000000000ULL;
            header.stamp.nsec = cloud.timestamp_ns % 1000000000ULL;
            header.frame_id = frame_id_;
            cv_bridge::CvImage cv_img(header, "mono8", frame.intensity_image);
            it_img_pub_.publish(cv_img.toImageMsg());
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
        double max_diff = 1.0 / double(std::max(1.0, double(LIDAR_FRAME_RATE*0.9)));
        auto closest_it = buffer_.findClosest(timestamp, max_diff);
            
        if (closest_it == nullptr) {
            logger_->error_stream() << "[LADriver::service_cb] No PointCloud found in buffer with timestamp constraint provided.";
            response.success = false;
            return false;
        }
        
        closest_it->setReferencePair(request.reference_pair);
        auto gnss_ptr = gnss_data_buffer_.findClosest(timestamp);
        if (gnss_ptr) { closest_it->addGNSSData(gnss_ptr->gnss_data); }

        auto odom_ptr = odom_data_buffer_.findClosest(timestamp);
        if (odom_ptr) { closest_it->addOdomData(odom_ptr->odom_data); }
        
        if (request.store) { closest_it->save(pc_path_, intensity_path_, request.reference_pair); }
        
        response.success = true;
        return true;
    }

    void gnss_data_cb(const sensor_msgs::NavSatFix::ConstPtr& msg)
    {
        std::vector<float> cov(9);
        std::copy(msg->position_covariance.data(), msg->position_covariance.data() + 9, cov.begin());
        gnss_data_buffer_.addFrame(GNSSDataBuffer{msg->header.stamp.toNSec(), GNSSMetadata(msg->latitude, msg->longitude, msg->altitude, cov)});
    }

    void odom_data_cb(const nav_msgs::Odometry::ConstPtr& msg) {
        std::vector<float> cov(36);
        std::copy(msg->pose.covariance.data(), msg->pose.covariance.data() + 36, cov.begin());
        odom_data_buffer_.addFrame(OdomDataBuffer{msg->header.stamp.toNSec(), 
            OdomMetadata(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z,
                         msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w,
                         cov)
        }); 
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