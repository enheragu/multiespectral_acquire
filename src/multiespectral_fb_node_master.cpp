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
    int current_frame_rate = 0;
public:

    MultiespectralAcquire(std::string name, std::string img_path) :
        as_(nh_, name, boost::bind(&MultiespectralAcquire::executeCB, this, _1), false),
        action_name_(name),
        MultiespectralAcquireT(img_path)
    {
        as_.start();

        image_transport::ImageTransport it(nh_);
    }

    bool init(int frame_rate)
    {
        current_frame_rate = frame_rate;
        bool result = MultiespectralAcquireT::init(frame_rate);
        result = result && setAsMaster();

        ROS_FATAL_STREAM_COND(!result, "[MultiespectralAcquire::init] Could not configure " << getName() << " camera as master.");
        ROS_INFO_STREAM_COND(result, "[MultiespectralAcquire::init] Initialized " << getName() << " camera as master.");
        
        result = result && beginAcquisition();

        return result;
    }

    void executeCB(const multiespectral_fb::MultiespectralAcquisitionGoalConstPtr &goal)
    { 
        feedback_.images_acquired = 0;
        feedback_.storage_path = "";
        ROS_INFO_STREAM("[MultiespectralAcquire::executeCB] Start image acquisition loop. " << std::string(goal->store?"S":"Not s") << "toring images. Frame rate is "<<std::to_string(current_frame_rate));

        if (goal->store)
        {
            feedback_.storage_path = img_path;
            ROS_INFO_STREAM("[MultiespectralAcquire::executeCB] Storing images to " << img_path);
        }

        bool result = true;
        
        ros::Rate loop_rate(current_frame_rate);
        while(ros::ok())
        {
            cv::Mat curr_image;
            result = this->grabStoreImage(curr_image, goal->store);
            if (result) 
            {
                feedback_.images_acquired = feedback_.images_acquired + 1;
                
                if (!curr_image.empty())
                {
                    feedback_.images_acquired = feedback_.images_acquired + 1;
                }
                as_.publishFeedback(feedback_);
            }
            
            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                as_.setPreempted();
                break;
            }
            loop_rate.sleep();
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
    ros::init(argc, argv, "MultiespectralMasterAcquire_" + getType());

    int frame_rate;
    ros::param::param<std::string>("~dataset_output_path", IMAGE_PATH, "./");
    ros::param::param<int>("~frame_rate", frame_rate, 10);

    std::string path = IMAGE_PATH+std::string("/")+getType()+std::string("/");
    std::filesystem::create_directories(IMAGE_PATH+std::string("/")+getType());

    ROS_INFO_STREAM("[MultiespectralAcquire::"<<getType()<<"::main] Images will be stored in path: " << path);
    std::shared_ptr<MultiespectralAcquire> camera_handler_ptr;
    camera_handler_ptr = std::make_shared<MultiespectralAcquire>("MultiespectralAcquire_" + getType(), path);
    bool result = camera_handler_ptr->init(frame_rate);
    
    if (result) ros::spin();

    return 0;
}