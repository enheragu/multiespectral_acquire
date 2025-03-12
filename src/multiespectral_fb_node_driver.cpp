#include <thread>
#include <signal.h>
#include <memory>
#include <filesystem>
#include <deque>

#include "ros/ros.h"

#include "camera_adapter.h"
#include <multiespectral_fb/ImageRequest.h>
std::string IMAGE_PATH; 
std::string IMAGE_TOPIC;
std::string CAMERA_IP;

int FLIR_FRAME_RATE = 30;
const double INTERVAL_BETWEEN_FRAMES_S = 1.0 / double(FLIR_FRAME_RATE+1); // max interval in seconds. ADds extra frame as epsilon

class MultiespectralAcquire : public MultiespectralAcquireT
{
protected:
    int current_frame_rate = 0;
    
    // Circular buffer to store images to select closest with timestamp
    std::deque<std::pair<uint64_t, cv::Mat>> image_buffer;
    size_t buffer_size = 1; // Tamaño del buffer 

public:
    MultiespectralAcquire(std::string name, std::string img_path, std::string topic_name): MultiespectralAcquireT(img_path, topic_name)
    {
        // Eliminar el servicio
    }

    bool init(int frame_rate, std::string camera_ip)
    {
        current_frame_rate = frame_rate;
        this->buffer_size = (int(FLIR_FRAME_RATE/current_frame_rate) + 1)*3;
        
        bool result = MultiespectralAcquireT::init(frame_rate, camera_ip);
        // result = result && setAsSlave();

        ROS_FATAL_STREAM_COND(!result, "[MASlave::init] Could not configure " << getName() << " camera as continuous driver.");
        ROS_INFO_STREAM_COND(result, "[MASlave::init] Initialized " << getName() << " camera as continuous driver.");

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
            result = this->grabStoreImage(curr_image, timestamp, false);
            
            ros::spinOnce();
            loop_rate.sleep();
        }
        return result;
    }


}; // End class MultiespectralAcquire

void sigintHandler(int dummy)
{
    ROS_INFO_STREAM("SIGNIT recieved, exiting program.");
    ros::shutdown();
    // camera_handler_ptr.reset();
}

int main(int argc, char **argv)
{
    signal(SIGINT, sigintHandler);
    ros::init(argc, argv, "MultiespectralSlaveAcquire_" + getType());

    int frame_rate;
    ros::param::param<std::string>("~dataset_output_path", IMAGE_PATH, "./");
    ros::param::param<std::string>("~image_topic", IMAGE_TOPIC, getType()+"_image");
    ros::param::param<int>("~frame_rate", frame_rate, 10);
    ros::param::param<std::string>("~camera_ip", CAMERA_IP, "");

    std::string path = IMAGE_PATH+std::string("/")+getType()+std::string("/");
    std::filesystem::create_directories(path);

    ROS_INFO_STREAM("[MASlave::main] Images will be stored in path: " << path);
    std::shared_ptr<MultiespectralAcquire> camera_handler_ptr;
    camera_handler_ptr = std::make_shared<MultiespectralAcquire>("MultiespectralAcquire", path, IMAGE_TOPIC);
    bool result = camera_handler_ptr->init(frame_rate, CAMERA_IP);
   
    // Iniciar adquisición continua directamente
    if (result) camera_handler_ptr->execute();

    ros::AsyncSpinner spinner(2); // 2 hilos para manejar callbacks
    spinner.start();

    ros::waitForShutdown();

    return 0;
}
