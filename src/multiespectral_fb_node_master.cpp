/**
 * @file    multiespectral_fb_node_master.cpp
 * @author  enheragu (e.heredia@umh.es)
 * @version 0.1
 * @date    2023-11-17
 * @brief   ROS Action server to handle camera acquisition as master (synchronized with a hardware)
 *          triggered slave. Note that the action never ends once started unless preempted manually.
 */


#include <signal.h>
#include <memory>
#include <filesystem>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <multiespectral_fb/MultiespectralAcquisitionAction.h>

#include "camera_adapter.h"

std::string IMAGE_PATH; 

// class MultiespectralAcquire;
// std::shared_ptr<MultiespectralAcquire> camera_handler_ptr;

class MultiespectralAcquire : public MultiespectralAcquireT
{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<multiespectral_fb::MultiespectralAcquisitionAction> as_;
    std::string action_name_;
    multiespectral_fb::MultiespectralAcquisitionFeedback feedback_;

public:

    MultiespectralAcquire(std::string name, std::string img_path) :
        as_(nh_, name, boost::bind(&MultiespectralAcquire::executeCB, this, _1), false),
        action_name_(name),
        MultiespectralAcquireT(img_path)
    {
        as_.start();
    }

    bool init(int frame_rate)
    {
        bool result = MultiespectralAcquireT::init(frame_rate);
        result = result && setAsMaster();

        ROS_FATAL_STREAM_COND(!result, "[MultiespectralAcquire::init] Could not configure " << getName() << " camera as master.");
        ROS_INFO_STREAM_COND(result, "[MultiespectralAcquire::init] Initialized " << getName() << " camera as master.");
        
        result = result && beginAcquisition();

        return result;
    }

    void executeCB(const multiespectral_fb::MultiespectralAcquisitionGoalConstPtr &goal)
    { 
        // init images acquired counter
        feedback_.images_acquired = 0;

        // helper variables
        bool result = true;

        // start image acquisition
        cv::Mat curr_image;
        ROS_INFO_STREAM("[MultiespectralAcquire::executeCB] Start image acquisition loop.");
        while(ros::ok())
        {
            result = this->grabStoreImage();
            if (result) feedback_.images_acquired = feedback_.images_acquired + 1;
            
            // check that preempt has not been requested by the client
            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                as_.setPreempted(); // set the action state to preempted
                break;
            }

            // publish the feedback
            as_.publishFeedback(feedback_);
        }
    }
}; // End class MultiespectralAcquire


void sigintHandler(int dummy)
{
    ROS_INFO_STREAM("SIGNIT recieved, exiting program.");
    ros::shutdown();
    // camera_handler_ptr.reset();
}

int main(int argc, char** argv)
{
    signal(SIGINT, sigintHandler);
    ros::init(argc, argv, "MultiespectralMasterAcquire");

    int frame_rate;
    ros::param::param<std::string>("dataset_output_path", IMAGE_PATH, "./");
    ros::param::param<int>("frame_rate", frame_rate, 10);

    std::string path = IMAGE_PATH+std::string("/")+getType()+std::string("/");
    std::filesystem::create_directories(IMAGE_PATH+std::string("/")+getType());

    ROS_INFO_STREAM("Images will be stored in path: " << path);
    std::shared_ptr<MultiespectralAcquire> camera_handler_ptr;
    camera_handler_ptr = std::make_shared<MultiespectralAcquire>("MultiespectralAcquire", path);
    bool result = camera_handler_ptr->init(frame_rate);
    
    if (result) ros::spin();

    return 0;
}