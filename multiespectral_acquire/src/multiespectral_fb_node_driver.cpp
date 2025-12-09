#include <thread>
#include <signal.h>
#include <memory>
#include <deque>

#include "rclcpp/rclcpp.hpp"

#include "camera_drivers/camera_adapter.h"


int FLIR_FRAME_RATE = 30;
const double INTERVAL_BETWEEN_FRAMES_S = 1.0 / double(FLIR_FRAME_RATE+1); // max interval in seconds. ADds extra frame as epsilon

class MultiespectralAcquire: public MultiespectralAcquireT
{
private:
    rclcpp::TimerBase::SharedPtr timer_;

    
    // Circular buffer to store images to select closest with timestamp

public:
    MultiespectralAcquire(std::string name): MultiespectralAcquireT(name)
    {
        this->init(this->getFrameRate());        
    }

    bool init(int frame_rate)
    {
        this->frame_rate = frame_rate;
        bool result = MultiespectralAcquireT::init(frame_rate);
        // result = result && setAsSlave();

        if(!result) RCLCPP_FATAL_STREAM(get_logger(),"[MADriver::init] Could not configure " << getName() << " camera as continuous driver.");
        if(result) RCLCPP_INFO_STREAM(get_logger(),"[MADriver::init] Initialized " << getName() << " camera as continuous driver.");

        result = result && beginAcquisition();
        
        if(result) RCLCPP_INFO_STREAM(get_logger(),SUCCEED_F << "[MADriver::init] Start image acquisition loop for camera "  << getName() << "." << RESET_F);
        if(!result) RCLCPP_FATAL_STREAM(get_logger(),"[MADriver::init] Camera init image acquisition failed");

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0/FLIR_FRAME_RATE),
            std::bind(&MultiespectralAcquire::acquisition_loop, this));
            
        if (!result) 
        {
            RCLCPP_FATAL_STREAM(get_logger(),"[MADriver::MADriver] Camera init failed");
            throw std::runtime_error("[MADriver::MADriver] Camera init failed");
        }
        RCLCPP_INFO_STREAM(get_logger(),SUCCEED_F << "[MADriver::MADriver] Camera "<<getName()<<" ("<<getType()<<") initialized successfully"<<RESET_F);
        return result;
    }
    
private:
    void acquisition_loop() {
        cv::Mat curr_image(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));  // Init given pattern to check
        createTestPattern(curr_image);
        ImageMetadata metadata;
        bool result = this->grabPublishImage(curr_image, metadata);
        if (!result) 
        {
            RCLCPP_WARN_STREAM(get_logger(),"[MADriver::acquisition_loop] Could not grab image from camera " << getName() << ".");
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
