
#include <thread>
#include <signal.h>
#include <memory>
#include <filesystem>
#include <deque>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"

#include "camera_drivers/camera_adapter.h"
#include "multiespectral_acquire/srv/image_request.hpp"

using ImageRequest = multiespectral_acquire::srv::ImageRequest;

std::string IMAGE_PATH; 
std::string IMAGE_TOPIC;
std::string CAMERA_IP;

// class MultiespectralAcquire;
// std::shared_ptr<MultiespectralAcquire> camera_handler_ptr;
int FLIR_FRAME_RATE = 30;
const double INTERVAL_BETWEEN_FRAMES_S = 1.0 / double(FLIR_FRAME_RATE+1); // max interval in seconds. ADds extra frame as epsilon

class MultiespectralAcquire : public MultiespectralAcquireT
{
private:
    rclcpp::TimerBase::SharedPtr timer_;

protected:
    rclcpp::Service<ImageRequest>::SharedPtr service_;
    
    // Circular buffer to store images to select closest with timestamp
    struct FrameData {
        uint64_t timestamp;
        cv::Mat image;
        ImageMetadata metadata;
    };  
    std::deque<FrameData> image_buffer;

    size_t buffer_size = 1; // Tamaño del buffer 

public:
    
    MultiespectralAcquire(std::string name): MultiespectralAcquireT(name)
    {
        service_ = this->create_service<ImageRequest>("multiespectral_slave_service", std::bind(&MultiespectralAcquire::service_cb, this,std::placeholders::_1, std::placeholders::_2));
       
    }

    bool init(int frame_rate)
    {
        this->frame_rate = frame_rate;
        int current_frame_rate = std::min(this->frame_rate, 1);
        this->buffer_size = (int(FLIR_FRAME_RATE/current_frame_rate) + 1)*3;
        
        bool result = MultiespectralAcquireT::init(frame_rate);
        // result = result && setAsSlave();

        if(!result) RCLCPP_FATAL_STREAM(get_logger(),"[MASlave::init] Could not configure " << getName() << " camera as slave.");
        if(result) RCLCPP_INFO_STREAM(get_logger(),"[MASlave::init] Initialized " << getName() << " camera as slave.");

        result = result && beginAcquisition();
        

        RCLCPP_INFO_STREAM(get_logger(),"[MASlave::init] Start image acquisition loop for camera "  << getName() << ".");
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0/FLIR_FRAME_RATE),
            std::bind(&MultiespectralAcquire::acquisition_loop, this));

        if (!result) 
        {
            RCLCPP_FATAL_STREAM(get_logger(),"[MASlave::MASlave] Camera init failed");
            throw std::runtime_error("[MASlave::MASlave] Camera init failed");
        }
        RCLCPP_INFO_STREAM(get_logger(),"[MASlave::MASlave] Camera initialized successfully");
        return result;
    }

    bool service_cb(const std::shared_ptr<ImageRequest::Request> request, std::shared_ptr<ImageRequest::Response> response)
    {
        // RCLCPP_INFO("[MASlave::service_cb] Recieved request to get closest image to: %lu", req.timestamp);
        bool ret = false;
        uint64_t timestamp = request->timestamp; 
        
        if (image_buffer.empty())
        {
            RCLCPP_WARN_STREAM(get_logger(),"[MASlave::service_cb] Buffer is still empty.");
            response->success = false;
            return false;
        }
        else
        {
            auto closest_it = std::min_element(image_buffer.begin(), image_buffer.end(),
                [timestamp](const FrameData& a, const FrameData& b) {
                    return std::abs(static_cast<int64_t>(a.timestamp - timestamp)) <
                        std::abs(static_cast<int64_t>(b.timestamp - timestamp));
                });

            ret = processImage(closest_it->image, closest_it->timestamp, closest_it->metadata, request->store);
            
            double time_diff_s = std::abs(static_cast<int64_t>(closest_it->timestamp - timestamp)) / 1e9; // Nanoseconds to seconds conversion
            RCLCPP_INFO_STREAM(get_logger(),"[MASlave::service_cb] Closest image to Basler has a time difference of " << time_diff_s << " seconds.");
            if (time_diff_s > INTERVAL_BETWEEN_FRAMES_S)
            {
                RCLCPP_WARN_STREAM(get_logger(),"[MASlave::service_cb] Closest image to " << timestamp << " is " << closest_it->timestamp << "; time difference: " << time_diff_s << " is greater than interval betweem frames ("<<INTERVAL_BETWEEN_FRAMES_S<<").");
                response->success = false;
                return true;
            }
        }

        response->success = ret;
        return ret;
    }

    void addImageToBuffer(const cv::Mat& image, uint64_t timestamp, ImageMetadata& metadata)
    {
        if (image_buffer.size() >= this->buffer_size)
        { 
            image_buffer.pop_front();
        }
        image_buffer.push_back({timestamp, image, metadata});
    }

private:
    bool acquisition_loop()
    {
        cv::Mat curr_image(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));  // Init given pattern to check
        createTestPattern(curr_image);
        uint64_t timestamp;
        ImageMetadata metadata;
        bool result = this->grabImage(curr_image, timestamp, metadata);
        if (result && !curr_image.empty()) 
        {
            // cv::imshow("Imagen", curr_image);
            // cv::waitKey(0); // Esperar a que se presione una tecla para cerrar la ventana
            addImageToBuffer(curr_image, timestamp, metadata);
        }
        return result;
    }

}; // End class MultiespectralAcquire

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    std::cout << "[multiespectral_fb_node_slave] Starting Multiespectral Acquire Slave Node for "<<getType()<<" images." << std::endl;
    auto node = std::make_shared<MultiespectralAcquire>("MultiespectralAcquire_Slave_" + getType());
    node->init(node->getFrameRate());
    rclcpp::spin(node);
    rclcpp::shutdown();
}
