#include <ros/ros.h>
#include <memory>
#include <deque>
#include <algorithm>
#include "camera_adapter_ros.h"
#include "multiespectral_acquire/ImageRequest.h"
#include "utils/image_metadata.h"
#include "utils/timed_frame_buffer.h"


int FLIR_FRAME_RATE = 30;

class MultiespectralAcquire : public CameraAdapterROS
{
protected:
    ros::ServiceServer service_;
    ros::ServiceClient lidar_client_;
    struct FrameData {
        uint64_t timestamp;
        cv::Mat image;
        ImageMetadata metadata;
    };
    // circular buffer to store images to select closest with timestamp
    TimedFrameBuffer<FrameData> image_buffer_;
public:
    MultiespectralAcquire(const std::string& name) : CameraAdapterROS(name)
    {
        // Needs the acquisition publisher to be executed at FLIR_FRAME_RATE :)
        this->init_and_start_acquisition(FLIR_FRAME_RATE); //(this->getFrameRate());

        service_ = nh_.advertiseService("multiespectral_slave_service", &MultiespectralAcquire::service_cb, this);
        lidar_client_ = nh_.serviceClient<multiespectral_acquire::ImageRequest>("lidar_slave_service");
    }
    bool service_cb(multiespectral_acquire::ImageRequest::Request &request, multiespectral_acquire::ImageRequest::Response &response) {
        bool ret = false;
        uint64_t timestamp = request.timestamp;
        if (image_buffer_.empty())
        {
            logger_->warn_stream() << "[MASlave::service_cb] Buffer is still empty.";
            response.success = false;
            return false;
        }
        else
        {
            // Gets closest (higher or lower)
            double max_diff = 1.0 / double(std::max(1.0, double(FLIR_FRAME_RATE*0.9)));
            auto closest_it = image_buffer_.findClosest(timestamp, max_diff);
            
            if (closest_it == nullptr)
            {
                logger_->error_stream() << "[MASlave::service_cb] No image found in buffer with timestamp constraint provided.";
                response.success = false;
                return true;
            }

            auto gnss_ptr = gnss_data_buffer_.findClosest(timestamp);
            if (gnss_ptr) { closest_it->metadata.addGNSSData(gnss_ptr->gnss_data); }

            auto odom_ptr = odom_data_buffer_.findClosest(timestamp);
            if (odom_ptr) { closest_it->metadata.addOdomData(odom_ptr->odom_data); }

            closest_it->metadata.img_pair_name = request.reference_pair;

            // only publishes and stores image if the time constraints are met
            ret = publishImage(closest_it->image, closest_it->metadata);
            if(ret && request.store)
            {
                ret = ret && storeImage(closest_it->image, closest_it->metadata);
            }
            
            // Try to call LIDAR service to store corresponding pointcloud and intensity image
            if (lidar_client_.exists()) {
                multiespectral_acquire::ImageRequest lidar_srv;
                lidar_srv.request.timestamp = closest_it->timestamp; // Both LWIR and LIDAR use PTP, so this one should be the best timestamp
                lidar_srv.request.reference_pair = request.reference_pair;
                lidar_srv.request.store = request.store;
                if (!lidar_client_.call(lidar_srv)) {
                    logger_->warn_stream() << "[MASlave::service_cb] LIDAR service call failed.";
                } else if (!lidar_srv.response.success) {
                    logger_->warn_stream() << "[MASlave::service_cb] LIDAR service responded but failed.";
                }
            } else {
                logger_->warn_stream() << "[MASlave::service_cb] LIDAR service not available.";
            }
            
        }
        response.success = ret;
        return ret;
    }

    void acquisition_loop(const ros::TimerEvent&) override
    {
        cv::Mat curr_image(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));  // Init given pattern to check
        createTestPattern(curr_image);
        ImageMetadata metadata;
        metadata.dataset_name = this->dataset_name;
        metadata.setROSTimeNowCallback([]() { return static_cast<uint64_t>(ros::Time::now().toNSec()); });
        bool result = this->grabImage(curr_image, metadata);
        if (result && !curr_image.empty())
        {
            // cv::imshow("Imagen", curr_image);
            // cv::waitKey(0); // Esperar a que se presione una tecla para cerrar la ventana
            image_buffer_.addFrame(FrameData({metadata.getSyncTimestamp(), curr_image, metadata}));
        }
    }

}; // End class MultiespectralAcquire

int main(int argc, char **argv) {
    ros::init(argc, argv, "multiespectral_fb_node_slave");
    ROS_INFO_STREAM("[multiespectral_fb_node_slave] Starting Multiespectral Acquire Slave Node for " << getType() << " images.");
    std::shared_ptr<MultiespectralAcquire> node = std::make_shared<MultiespectralAcquire>("MultiespectralAcquire_Slave_" + getType());
    ros::spin();
    return 0;
}
