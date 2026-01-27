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
#include <actionlib/client/simple_action_client.h>
#include "multiespectral_acquire/MultiespectralAcquisitionAction.h"
#include "multiespectral_acquire/ImageRequest.h"
#include "camera_adapter_ros.h"
#include "utils/image_metadata.h"

std::string IMAGE_PATH; 
std::string IMAGE_TOPIC;
std::string CAMERA_IP;

class MultiespectralAcquire : public CameraAdapterROS {
protected:
    actionlib::SimpleActionServer<multiespectral_acquire::MultiespectralAcquisitionAction> action_server_;
    std::string action_name_;
    ros::ServiceClient slave_camera_client_;
    multiespectral_acquire::MultiespectralAcquisitionFeedback feedback_;
    multiespectral_acquire::MultiespectralAcquisitionResult result_;
    std::mutex goal_mutex_;
public:
    MultiespectralAcquire(const std::string& name)
        : CameraAdapterROS(name),
          action_server_(nh_, "AS", boost::bind(&MultiespectralAcquire::execute, this, _1), false),
          action_name_(name)
    {
        this->init_and_start_acquisition(this->getFrameRate());
        slave_camera_client_ = nh_.serviceClient<multiespectral_acquire::ImageRequest>("multiespectral_slave_service");
        action_server_.start();
    }

    void execute(const multiespectral_acquire::MultiespectralAcquisitionGoalConstPtr& goal) {
        feedback_.images_acquired = 0;
        feedback_.storage_path = "none";
        logger_->info_stream() << "[MAMaster::execute] Start image acquisition loop. " << (goal->store?"S":"Not s") << "toring images. Frame rate is " << this->frame_rate << "Hz for camera " << getName() << ".";
        if (goal->store) {
            feedback_.storage_path = img_path;
            logger_->info_stream() << "[MAMaster::execute] Storing images to " << img_path;
        }
        ros::Rate loop_rate(this->frame_rate);
        bool result = true;
        while (ros::ok() && action_server_.isActive()) {
            if (action_server_.isPreemptRequested()) {
                action_server_.setPreempted();
                logger_->info_stream() << "Goal canceled.";
                return;
            }
            cv::Mat curr_image(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
            createTestPattern(curr_image);

            ImageMetadata metadata;
            metadata.dataset_name = this->dataset_name;
            metadata.setROSTimeNowCallback([]() { return static_cast<uint64_t>(ros::Time::now().toNSec()); });
            logger_->debug_stream() << "[MAMaster::executeCB] Grabbing image.";
            result = this->grabPublishImage(curr_image, metadata);
            
            uint64_t timestamp = metadata.getSyncTimestamp();
            auto gnss_ptr = gnss_data_buffer_.findClosest(timestamp);
            if (gnss_ptr) { metadata.addGNSSData(gnss_ptr->gnss_data); }

            auto odom_ptr = odom_data_buffer_.findClosest(timestamp);
            if (odom_ptr) { metadata.addOdomData(odom_ptr->odom_data); }

            if(result && goal->store)
            {
                logger_->debug_stream() << "[MAMaster::executeCB] Storing image.";
                result = this->storeImage(curr_image, metadata);
            }
            if (result && !curr_image.empty())
            {
                feedback_.images_acquired++;
                result_.images_acquired = feedback_.images_acquired;
                action_server_.publishFeedback(feedback_);
            }
            // Petición al slave
            multiespectral_acquire::ImageRequest srv;
            srv.request.timestamp = timestamp;
            srv.request.store = goal->store;
            srv.request.reference_pair = metadata.img_name;
            if (!slave_camera_client_.call(srv)) {
                logger_->error_stream() << "Slave camera service not available or call failed";
                action_server_.setAborted(result_);
                return;
            }
            loop_rate.sleep();
        }
        action_server_.setSucceeded(result_);
    }
    
    // Override required by virtual method in CameraAdapterROS
    void acquisition_loop(const ros::TimerEvent&) override {
        // Not used, the master uses its own loop in execute()
    }

}; // End class MultiespectralAcquire



int main(int argc, char **argv) {
    ros::init(argc, argv, "multiespectral_fb_node_master");
    ROS_INFO_STREAM("[multiespectral_fb_node_master] Starting Multiespectral Acquire Master Node for " << getType() << " images.");
    std::shared_ptr<MultiespectralAcquire> node = std::make_shared<MultiespectralAcquire>("MultiespectralAcquire_Master_" + getType());
    ros::spin();
    return 0;
}

