

#include <signal.h>
#include <memory>
#include <filesystem>

#include "ros/ros.h"

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "camera_adapter.h"
std::string IMAGE_PATH; 

// class MultiespectralAcquire;
// std::shared_ptr<MultiespectralAcquire> camera_handler_ptr;

class MultiespectralAcquire : public MultiespectralAcquireT
{
private:
    ros::NodeHandle nh_;
    image_transport::Publisher image_pub_;

public:
    MultiespectralAcquire(std::string name, std::string img_path): MultiespectralAcquireT(img_path) 
    {
        image_transport::ImageTransport it(nh_);
        image_pub_ = it.advertise(getType()+"_image", 1);
    }

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

            if (result && !curr_image.empty())
            {
                // Convert to a sensor_msgs::Image message
                sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", curr_image).toImageMsg();
                image_pub_.publish(msg);
            }

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
    ros::param::param<std::string>("/basler_multiespectral/dataset_output_path", IMAGE_PATH, "./");
    ros::param::param<int>("/basler_multiespectral/frame_rate", frame_rate, 10);

    std::string path = IMAGE_PATH+std::string("/")+getType()+std::string("/");
    std::filesystem::create_directories(path);

    ROS_INFO_STREAM("Images will be stored in path: " << path);
    std::shared_ptr<MultiespectralAcquire> camera_handler_ptr;
    camera_handler_ptr = std::make_shared<MultiespectralAcquire>("MultiespectralAcquire", path);
    bool result = camera_handler_ptr->init(frame_rate);
   
    if (result) camera_handler_ptr->execute();

    return 0;
}
