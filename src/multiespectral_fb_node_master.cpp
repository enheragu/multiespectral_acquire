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
#include <multiespectral_fb/ImageRequest.h>

#include "camera_adapter.h"

std::string IMAGE_PATH; 
std::string IMAGE_TOPIC;
std::string CAMERA_IP;

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

    ros::ServiceClient slave_camera_client_;
public:

    MultiespectralAcquire(std::string name, std::string img_path, std::string topic_name) :
        as_(nh_, name, boost::bind(&MultiespectralAcquire::executeCB, this, _1), false),
        action_name_(name),
        MultiespectralAcquireT(img_path, topic_name)
    {
        as_.start();

        image_transport::ImageTransport it(nh_);

        slave_camera_client_ = nh_.serviceClient<multiespectral_fb::ImageRequest>("multiespectral_slave_service");
    }

    bool init(int frame_rate, std::string camera_ip)
    {
        current_frame_rate = frame_rate;
        bool result = MultiespectralAcquireT::init(frame_rate, camera_ip);
        // result = result && setAsMaster();

        ROS_FATAL_STREAM_COND(!result, "[MAMaster::init] Could not configure " << getName() << " camera as master.");
        ROS_INFO_STREAM_COND(result, "[MAMaster::init] Initialized " << getName() << " camera as master with "<<current_frame_rate<<"Hz frame rate.");
        
        result = result && beginAcquisition();

        return result;
    }

    void executeCB(const multiespectral_fb::MultiespectralAcquisitionGoalConstPtr &goal)
    { 
        feedback_.images_acquired = 0;
        feedback_.storage_path = "";
        ROS_INFO_STREAM("[MAMaster::executeCB] Start image acquisition loop. " << std::string(goal->store?"S":"Not s") << "toring images. Frame rate is "<<std::to_string(current_frame_rate) << "Hz for camera " << getName() << ".");

        if (goal->store)
        {
            feedback_.storage_path = img_path;
            ROS_INFO_STREAM("[MAMaster::executeCB] Storing images to " << img_path);
        }

        bool result = true;
        
        ros::Rate loop_rate(current_frame_rate);
        while(ros::ok())
        {
            cv::Mat curr_image;
            uint64_t timestamp;
            ImageMetadata metadata;
            // ROS_INFO("[MAMaster::executeCB] Grabbing image.");
            result = this->grabStoreImage(curr_image, timestamp, metadata, goal->store);
            if (result) 
            {
                feedback_.images_acquired = feedback_.images_acquired + 1;
                
                if (!curr_image.empty())
                {
                    feedback_.images_acquired = feedback_.images_acquired + 1;
                }
                as_.publishFeedback(feedback_);
            }

            // ROS_INFO("[MAMaster::executeCB] Prepare request for image with timestamp: %lu", timestamp);
            multiespectral_fb::ImageRequest srv;
            srv.request.timestamp = timestamp;
            srv.request.store = goal->store;
            if (!slave_camera_client_.call(srv))
            {
                ROS_ERROR_STREAM("Could not contact with slave service to request image with current timestamp: " << timestamp);
            }
            // ROS_INFO("[MAMaster::executeCB] Requested image with timestamp: %lu", timestamp);
            
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
    std::string node_name = ros::this_node::getName();

    int frame_rate;
    ros::param::param<std::string>("~dataset_output_path", IMAGE_PATH, "./");
    ros::param::param<int>("~frame_rate", frame_rate, 10);
    ros::param::param<std::string>("~image_topic", IMAGE_TOPIC, getType()+"_image");
    ros::param::param<std::string>("~camera_ip", CAMERA_IP, "");

    std::string path = IMAGE_PATH+std::string("/")+getFolderTimetag()+std::string("/")+getType()+std::string("/");
    std::filesystem::create_directories(path);

    ROS_INFO_STREAM("[MAMaster::"<<getType()<<"::main] Images will be stored in path: " << path);
    std::shared_ptr<MultiespectralAcquire> camera_handler_ptr;
    camera_handler_ptr = std::make_shared<MultiespectralAcquire>("AS", path, IMAGE_TOPIC);
    bool result = camera_handler_ptr->init(frame_rate,CAMERA_IP);
    
    if (result) 
    {
        ros::spin();
        // ros::AsyncSpinner spinner(2); // 2 hilos para manejar callbacks
        // spinner.start();
    }

    return 0;
}