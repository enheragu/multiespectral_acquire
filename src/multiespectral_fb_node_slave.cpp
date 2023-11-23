

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
public:
    MultiespectralAcquire(std::string name, std::string img_path): MultiespectralAcquireT(img_path) {    }

    bool init(int frame_rate)
    {
        bool result = MultiespectralAcquireT::init(frame_rate);
        result = result && setAsSlave();

        ROS_FATAL_STREAM_COND(!result, "[MultiespectralMaster] Could not configure " << getName() << " camera as slave.");
        return result;
    }

    bool execute()
    {
        bool result = false;
        while (ros::ok())
        {
            result = this->grabStoreImage();
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
    ros::param::param<std::string>("dataset_output_path", IMAGE_PATH, "./");
    ros::param::param<int>("frame_rate", frame_rate, 1);

    std::filesystem::create_directories(IMAGE_PATH+std::string("/")+getType());

    std::shared_ptr<MultiespectralAcquire> camera_handler_ptr;
    camera_handler_ptr = std::make_shared<MultiespectralAcquire>("MultiespectralAcquire", IMAGE_PATH);
    bool result = camera_handler_ptr->init(frame_rate);
   
    if (result) camera_handler_ptr->execute();

    return 0;
}
