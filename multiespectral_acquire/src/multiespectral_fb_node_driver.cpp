#include <thread>
#include <signal.h>
#include <memory>
#include <deque>

#include <ros/ros.h>
#include "camera_drivers/camera_adapter.h"
#include "camera_adapter_ros.h"
#include "utils/image_metadata.h"


class MultiespectralAcquire: public CameraAdapterROS
{
public:
    MultiespectralAcquire(const std::string& name): CameraAdapterROS(name)
    {
        this->init_and_start_acquisition(this->getFrameRate());
    }

    
private:
    void acquisition_loop(const ros::TimerEvent&) override {
        cv::Mat curr_image(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));  // Init given pattern to check
        createTestPattern(curr_image);
        ImageMetadata metadata;
        metadata.dataset_name = this->dataset_name;
        metadata.setROSTimeNowCallback([]() { return static_cast<uint64_t>(ros::Time::now().toNSec()); });
        bool result = this->grabPublishImage(curr_image, metadata);
        if (!result) 
        {
            logger_->warn_stream() << "[MADriver::acquisition_loop] Could not grab image from camera " << getName() << ".";
            return;
        }
    }
}; // End class MultiespectralAcquire


int main(int argc, char **argv) {
    ros::init(argc, argv, "multiespectral_fb_node_driver");
    ROS_INFO_STREAM("[multiespectral_fb_node_driver] Starting Multiespectral Acquire Driver Node for " << getType() << " images.");
    std::shared_ptr<MultiespectralAcquire> node = std::make_shared<MultiespectralAcquire>("CameraAcquire_Driver_" + getType());
    ros::spin();
    return 0;
}
