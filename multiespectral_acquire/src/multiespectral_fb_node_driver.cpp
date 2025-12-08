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

protected:
    int current_frame_rate = 0;
    
    // Circular buffer to store images to select closest with timestamp
    std::deque<std::pair<uint64_t, cv::Mat>> image_buffer;
    size_t buffer_size = 1; // Tamaño del buffer 

public:
    MultiespectralAcquire(std::string name): MultiespectralAcquireT(name)
    {
    }

    bool init(int frame_rate)
    {
        current_frame_rate = frame_rate;
        this->buffer_size = (int(FLIR_FRAME_RATE/current_frame_rate) + 1)*3;
        
        bool result = MultiespectralAcquireT::init(frame_rate);
        // result = result && setAsSlave();

        if(!result) RCLCPP_FATAL_STREAM(get_logger(),"[MADriver::init] Could not configure " << getName() << " camera as continuous driver.");
        if(result) RCLCPP_INFO_STREAM(get_logger(),"[MADriver::init] Initialized " << getName() << " camera as continuous driver.");

        result = result && beginAcquisition();
        
        RCLCPP_INFO_STREAM(get_logger(),"[MADriver::init] Start image acquisition loop for camera "  << getName() << ".");
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0/FLIR_FRAME_RATE),
            std::bind(&MultiespectralAcquire::acquisition_loop, this));
            
        return result;
    }
    
private:
    void acquisition_loop() {
        cv::Mat curr_image;
        uint64_t timestamp;
        ImageMetadata metadata;
        bool result = this->grabStoreImage(curr_image, timestamp, metadata, false);
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
    auto node = std::make_shared<MultiespectralAcquire>("CameraDriverAcquire_" + getType());
    rclcpp::spin(node);
    rclcpp::shutdown();
}
