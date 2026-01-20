#include <thread>
#include <signal.h>
#include <memory>
#include <deque>

#include "rclcpp/rclcpp.hpp"

#include "camera_drivers/camera_adapter.h"
#include "camera_adapter_ros.h"
#include "utils/image_metadata.h"


int FLIR_FRAME_RATE = 30;
const double INTERVAL_BETWEEN_FRAMES_S = 1.0 / double(FLIR_FRAME_RATE+1); // max interval in seconds. ADds extra frame as epsilon

class MultiespectralAcquire: public CameraAdapterROS
{
public:
    MultiespectralAcquire(std::string name): CameraAdapterROS(name)
    {
        this->init_and_start_acquisition(this->getFrameRate());
    }

    
private:
    void acquisition_loop() {
        cv::Mat curr_image(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));  // Init given pattern to check
        createTestPattern(curr_image);
        ImageMetadata metadata;
        metadata.setROSTimeNowCallback([this]() { return this->get_clock()->now().nanoseconds(); });
        bool result = this->grabPublishImage(curr_image, metadata);
        if (!result) 
        {
            logger_->warn_stream() << "[MADriver::acquisition_loop] Could not grab image from camera " << getName() << ".";
            return;
        }
    }
}; // End class MultiespectralAcquire


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    std::cout << "[multiespectral_fb_node_driver] Starting Multiespectral Acquire Driver Node for "<<getType()<<" images." << std::endl;
    auto node = std::make_shared<MultiespectralAcquire>("CameraAcquire_Driver_" + getType());
    rclcpp::spin(node);
    rclcpp::shutdown();
}
