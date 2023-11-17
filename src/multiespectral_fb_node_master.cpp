/**
 * @file    multiespectral_fb_node_master.cpp
 * @author  enheragu (e.heredia@umh.es)
 * @version 0.1
 * @date    2023-11-17
 * @brief   ROS Action server to handle camera acquisition as master (synchronized with a hardware)
 *          triggered slave. Note that the action never ends once started unless preempted manually.
 */

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <multiespectral_fb/MultiespectralAcquisitionAction.h>

#include "camera_adapter.h"

std::string IMAGE_PATH; 

class MultiespectralAcquire
{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<multiespectral_fb::MultiespectralAcquisitionAction> as_;
    std::string action_name_;
    multiespectral_fb::MultiespectralAcquisitionFeedback feedback_;

public:

    MultiespectralAcquire(std::string name, int frame_rate) :
        as_(nh_, name, boost::bind(&MultiespectralAcquire::executeCB, this, _1), false),
        action_name_(name)
    {
        as_.start();
        bool result = initCamera(frame_rate);
        result = result | setAsMaster();

        ROS_FATAL_STREAM_COND(!result, "[MultiespectralMaster] Could not initialize " << getName() << " camera.");
        if (!result) ros::shutdown();
    }

    ~MultiespectralAcquire(void)
    {   
        bool result = closeCamera();
        ROS_FATAL_STREAM_COND(!result, "[MultiespectralMaster] Could finish correctly " << getName() << " camera.");
    }

    void executeCB(const multiespectral_fb::MultiespectralAcquisitionGoalConstPtr &goal)
    { 
        // init images acquired counter
        feedback_.images_acquired = 0;

        // helper variables
        bool result = true;

        // start image acquisition
        cv::Mat curr_image;
        while(true)
        {
            result =  acquireImage(curr_image);
            if (result) 
            {
                std::ostringstream filename;
                filename << IMAGE_PATH << getType() << std::to_string(feedback_.images_acquired) << ".jpg";
                cv::imwrite(filename.str().c_str(), curr_image);
                
                feedback_.images_acquired = feedback_.images_acquired + 1;
            }
            
            ROS_ERROR_STREAM_COND(!result, "[MultiespectralMaster::executeCB] Could not acquire image from " << getName() << " camera.");
            
            
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
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "MultiespectralMasterAcquire");

    /**
     * Prepare image folder
     */
    int frame_rate;
    ros::param::param<std::string>("dataset_output_path", IMAGE_PATH, "./");
    ros::param::param<int>("frame_rate", frame_rate, 1);


    MultiespectralAcquire acquire_class("MultiespectralAcquire", frame_rate);
    ros::spin();

    return 0;
}