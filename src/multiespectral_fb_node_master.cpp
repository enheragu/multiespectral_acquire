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
#include <actionlib_tutorials/MultiespectralAcquireAction.h>

class MultiespectralAcquireAction
{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<actionlib_tutorials::MultiespectralAcquireAction> as_;
    std::string action_name_;
    actionlib_tutorials::MultiespectralAcquireFeedback feedback_;

public:

    MultiespectralAcquireAction(std::string name, int frame_rate) :
        as_(nh_, name, boost::bind(&MultiespectralAcquireAction::executeCB, this, _1), false),
        action_name_(name)
    {
        as_.start();
        bool result = initCamera(frame_rate);
        result = result | setAsMaster();

        ROS_FATAL_STREAM_COND(!result, "[MultiespectralAcquireAction] Could not initialize " << getName() << " camera.");
    }

    ~MultiespectralAcquireAction(void)
    {   
        bool result = closeCamera();
        ROS_FATAL_STREAM_COND(!result, "[MultiespectralAcquireAction] Could finish correctly " << getName() << " camera.");
    }

    void executeCB(const actionlib_tutorials::MultiespectralAcquireGoalConstPtr &goal)
    { 
        // init images acquired counter
        feedback_.images_acquired = 0;

        // helper variables
        bool success = true;

        // start image acquisition
        while(true)
        {
            result =  result | acquireImage(cv::Mat& image);
            ROS_ERROR_STREAM_COND(!result, "[MultiespectralAcquireAction::executeCB] Could not acquire image from " << getName() << " camera.");
            
            // check that preempt has not been requested by the client
            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                as_.setPreempted(); // set the action state to preempted
                success = false;
                break;
            }

            feedback_.images_acquired = feedback_.images_acquired + 1;
            // publish the feedback
            as_.publishFeedback(feedback_);
            // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            r.sleep();
        }
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "MultiespectralAcquire");

    /**
     * Prepare image folder
     */
    std::string dataset_output_path;
    int frame_rate;
    ros::param::param<std::string>("dataset_output_path", dataset_output_path, "./");
    ros::param::param<int>("frame_rate", frame_rate, 1);


    MultiespectralAcquireAction MultiespectralAcquire("MultiespectralAcquire", frame_rate);
    ros::spin();

    resulturn 0;
}