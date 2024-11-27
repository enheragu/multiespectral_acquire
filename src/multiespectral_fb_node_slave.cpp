

#include <signal.h>
#include <memory>
#include <filesystem>

#include "ros/ros.h"

#include "camera_adapter.h"
std::string IMAGE_PATH; 

// class MultiespectralAcquire;
// std::shared_ptr<MultiespectralAcquire> camera_handler_ptr;

class MultiespectralAcquire : public MultiespectralAcquireT
{
private:

public:
    MultiespectralAcquire(std::string name, std::string img_path): MultiespectralAcquireT(img_path) {    }

    bool init(int frame_rate)
    {
        bool result = MultiespectralAcquireT::init(frame_rate);
        result = result && setAsSlave();

        ROS_FATAL_STREAM_COND(!result, "[MultiespectralAcquire::init] Could not configure " << getName() << " camera as slave.");
        ROS_INFO_STREAM_COND(result, "[MultiespectralAcquire::init] Initialized " << getName() << " camera as slave.");

        result = result && beginAcquisition();

        return result;
    }
    
    bool execute()
    {
        bool result = false;
        ROS_INFO_STREAM("[MultiespectralAcquire::execute] Start image acquisition loop.");
        while (ros::ok())
        {
            cv::Mat curr_image;
            result = this->grabStoreImage(curr_image);
            ros::spinOnce();
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
    ros::param::param<int>("~frame_rate", frame_rate, 10);

    std::string path = IMAGE_PATH+std::string("/")+getType()+std::string("/");
    std::filesystem::create_directories(path);

    ROS_INFO_STREAM("[MultiespectralAcquire::main] Images will be stored in path: " << path);
    std::shared_ptr<MultiespectralAcquire> camera_handler_ptr;
    camera_handler_ptr = std::make_shared<MultiespectralAcquire>("MultiespectralAcquire", path);
    bool result = camera_handler_ptr->init(frame_rate);
   
    if (result) camera_handler_ptr->execute();

    return 0;
}
