#include <ros/ros.h>
#include <memory>
#include <deque>
#include <algorithm>
#include "camera_adapter_ros.h"
#include "multiespectral_acquire/ImageRequest.h"
#include "utils/image_metadata.h"

int FLIR_FRAME_RATE = 30;
const double INTERVAL_BETWEEN_FRAMES_S = 1.0 / double(FLIR_FRAME_RATE-1); // max interval in seconds. Adds extra frame as epsilon

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
    std::deque<FrameData> image_buffer;
    size_t buffer_size = 1;
public:
    MultiespectralAcquire(const std::string& name) : CameraAdapterROS(name)
    {
        this->init_and_start_acquisition(this->getFrameRate());
        
        int current_frame_rate = std::max(this->frame_rate, 1);
        this->buffer_size = (int(FLIR_FRAME_RATE/current_frame_rate) + 1)*3;
        service_ = nh_.advertiseService("multiespectral_slave_service", &MultiespectralAcquire::service_cb, this);
        lidar_client_ = nh_.serviceClient<multiespectral_acquire::ImageRequest>("lidar_slave_service");
    }
    bool service_cb(multiespectral_acquire::ImageRequest::Request &request, multiespectral_acquire::ImageRequest::Response &response) {
        bool ret = false;
        uint64_t timestamp = request.timestamp;
        if (image_buffer.empty())
        {
            logger_->warn_stream() << "[MASlave::service_cb] Buffer is still empty.";
            response.success = false;
            return false;
        }
        else
        {
            // Gets closest (higher or lower)
            auto closest_it = std::min_element(image_buffer.begin(), image_buffer.end(),
                [timestamp](const FrameData& a, const FrameData& b) {
                    return std::abs(static_cast<int64_t>(a.timestamp - timestamp)) <
                        std::abs(static_cast<int64_t>(b.timestamp - timestamp));
                });
            
            // Gets closest (higher or equal)
            // auto closest_it = std::lower_bound(image_buffer.begin(), image_buffer.end(), timestamp,
            //     [](const FrameData& a, uint64_t ts) { return a.timestamp < ts; });
            
            if (closest_it == image_buffer.end())
            {
                logger_->error_stream() << "[MASlave::service_cb] No image found in buffer with timestamp constraint provided.";
            }
            
            closest_it->metadata.img_pair_name = request.reference_pair;
            
            double time_diff_s = std::abs(static_cast<int64_t>(closest_it->timestamp - timestamp)) / 1e9; // Nanoseconds to seconds conversion
            // logger_->info_stream() << "[MASlave::service_cb] Closest image found -> time difference: " << time_diff_s << " seconds.";
            if (time_diff_s > INTERVAL_BETWEEN_FRAMES_S)
            {
                logger_->warn_stream() << "[MASlave::service_cb] Closest image to " << timestamp << " is " << closest_it->timestamp << "; time difference: " << time_diff_s << " is greater than interval between frames ("<<INTERVAL_BETWEEN_FRAMES_S<<").";
                response.success = false;
                return true;
            }
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
    void addImageToBuffer(const cv::Mat& image, ImageMetadata& metadata) 
    {
        if (image_buffer.size() >= this->buffer_size)
        {
            image_buffer.pop_front();
        }
        image_buffer.push_back({metadata.getSyncTimestamp(), image, metadata});
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
            addImageToBuffer(curr_image, metadata);
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
