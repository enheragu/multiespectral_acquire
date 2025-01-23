
#include <thread>
#include <signal.h>
#include <memory>
#include <filesystem>
#include <deque>

#include "ros/ros.h"

#include "camera_adapter.h"
#include <multiespectral_fb/ImageRequest.h>
std::string IMAGE_PATH; 

// class MultiespectralAcquire;
// std::shared_ptr<MultiespectralAcquire> camera_handler_ptr;
int FLIR_FRAME_RATE = 30;
const double INTERVAL_BETWEEN_FRAMES_S = 1.0 / double(FLIR_FRAME_RATE+1); // max interval in seconds. ADds extra frame as epsilon

class MultiespectralAcquire : public MultiespectralAcquireT
{
protected:
    int current_frame_rate = 0;
    ros::ServiceServer service_;
    
    // Circular buffer to store images to select closest with timestamp
    std::deque<std::pair<uint64_t, cv::Mat>> image_buffer;
    size_t buffer_size = 1; // Tamaño del buffer 

public:
    MultiespectralAcquire(std::string name, std::string img_path): MultiespectralAcquireT(img_path)
    {
        service_ = nh_.advertiseService("multiespectral_slave_service", &MultiespectralAcquire::service_cb, this);
    }

    bool init(int frame_rate)
    {
        current_frame_rate = frame_rate;
        this->buffer_size = (int(FLIR_FRAME_RATE/current_frame_rate) + 1)*3;
        
        bool result = MultiespectralAcquireT::init(frame_rate);
        // result = result && setAsSlave();

        ROS_FATAL_STREAM_COND(!result, "[MASlave::init] Could not configure " << getName() << " camera as slave.");
        ROS_INFO_STREAM_COND(result, "[MASlave::init] Initialized " << getName() << " camera as slave.");

        result = result && beginAcquisition();

        return result;
    }
    
    bool execute()
    {
        bool result = false;
        ROS_INFO_STREAM("[MASlave::execute] Start image acquisition loop for camera "  << getName() << ".");

        ros::Rate loop_rate(FLIR_FRAME_RATE); // Flir camera goes at 30Hz all the time, make buffer to select closest image
        while (ros::ok())
        {
            cv::Mat curr_image;
            uint64_t timestamp;
            result = this->grabImage(curr_image, timestamp);
            if (result && !curr_image.empty()) 
            {
                // cv::imshow("Imagen", curr_image);
                // cv::waitKey(0); // Esperar a que se presione una tecla para cerrar la ventana

                addImageToBuffer(curr_image, timestamp);
            }
            ros::spinOnce();
            loop_rate.sleep();
        }
        return result;
    }


    bool service_cb(multiespectral_fb::ImageRequest::Request &req, multiespectral_fb::ImageRequest::Response &res)
    {
        // ROS_INFO("[MASlave::service_cb] Recieved request to get closest image to: %lu", req.timestamp);
        bool ret = false;
        uint64_t timestamp = req.timestamp; 
        
        if (image_buffer.empty())
        {
            ROS_WARN_STREAM("[MASlave::service_cb] Buffer is still empty.");
            res.success = false;
            return false;
        }
        else
        {
            auto closest_it = std::min_element(image_buffer.begin(), image_buffer.end(), [timestamp](const auto& a, const auto& b) 
                {
                    return std::abs(static_cast<int64_t>(a.first - timestamp)) < std::abs(static_cast<int64_t>(b.first - timestamp));
                }); 
                
            uint64_t closest_timestamp = closest_it->first;
            double time_diff_s = std::abs(static_cast<int64_t>(closest_timestamp - timestamp)) / 1e9; // Nanoseconds to seconds conversion

            cv::Mat closest_image = closest_it->second;
            ret = StoreImage(closest_image, timestamp, req.store);
            ROS_INFO_STREAM("[MASlave::service_cb] Closest image to Basler has a time difference of " << time_diff_s << " seconds.");
            if (time_diff_s > INTERVAL_BETWEEN_FRAMES_S)
            {
                ROS_WARN_STREAM("[MASlave::service_cb] Closest image to " << timestamp << " is " << closest_timestamp << "; time difference: " << time_diff_s << " is greater than interval betweem frames ("<<INTERVAL_BETWEEN_FRAMES_S<<").");
                res.success = false;
                return true;
            }
        }
        ros::spinOnce();
        res.success = ret;
        return ret;
    }

    void addImageToBuffer(const cv::Mat& image, uint64_t timestamp)
    {
        if (image_buffer.size() >= this->buffer_size)
        { 
            image_buffer.pop_front();
        }
        image_buffer.push_back(std::make_pair(timestamp, image));
    }


}; // End class MultiespectralAcquire

void sigintHandler(int dummy)
{
    ROS_INFO_STREAM("SIGNIT recieved, exiting program.");
    ros::shutdown();
    // camera_handler_ptr.reset();
}

void executeInThread(std::shared_ptr<MultiespectralAcquire> camera_handler_ptr)
{
    if (camera_handler_ptr) {
        camera_handler_ptr->execute();
    }
}

int main(int argc, char **argv)
{
    signal(SIGINT, sigintHandler);
    ros::init(argc, argv, "MultiespectralSlaveAcquire_" + getType());

    int frame_rate;
    ros::param::param<std::string>("~dataset_output_path", IMAGE_PATH, "./");
    ros::param::param<int>("~frame_rate", frame_rate, 10);

    std::string path = IMAGE_PATH+std::string("/")+getType()+std::string("/");
    std::filesystem::create_directories(path);

    ROS_INFO_STREAM("[MASlave::main] Images will be stored in path: " << path);
    std::shared_ptr<MultiespectralAcquire> camera_handler_ptr;
    camera_handler_ptr = std::make_shared<MultiespectralAcquire>("MultiespectralAcquire", path);
    bool result = camera_handler_ptr->init(frame_rate);
   
    // if (result) camera_handler_ptr->execute();
    std::thread camera_thread(executeInThread, camera_handler_ptr);
    ros::AsyncSpinner spinner(2); // 2 hilos para manejar callbacks
    spinner.start();

    ros::waitForShutdown();
    if (camera_thread.joinable()) {
        camera_thread.join();
    }


    return 0;
}
