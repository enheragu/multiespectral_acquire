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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "multiespectral_acquire/action/multiespectral_acquisition.hpp" 
#include "multiespectral_acquire/srv/image_request.hpp"

#include "camera_adapter_ros.h"
#include "utils/image_metadata.h"

std::string IMAGE_PATH; 
std::string IMAGE_TOPIC;
std::string CAMERA_IP;

// class MultiespectralAcquire;
// std::shared_ptr<MultiespectralAcquire> camera_handler_ptr;
using MultiespectralAcquisition = multiespectral_acquire::action::MultiespectralAcquisition;
using GoalHandle = rclcpp_action::ServerGoalHandle<MultiespectralAcquisition>;
using ImageRequest = multiespectral_acquire::srv::ImageRequest;
using MAGoalHandler = rclcpp_action::ServerGoalHandle<MultiespectralAcquisition>;

class MultiespectralAcquire : public CameraAdapterROS
{
protected:
    
    rclcpp_action::Server<MultiespectralAcquisition>::SharedPtr action_server_; 
    std::string action_name_;

    rclcpp::Client<ImageRequest>::SharedPtr slave_camera_client_;

    std::shared_ptr<MAGoalHandler> active_goal_;
    std::mutex goal_mutex_;
public:
    
    MultiespectralAcquire(std::string name): CameraAdapterROS(name), action_name_(name)
    {
        this->init_and_start_acquisition(this->getFrameRate());
        
        using namespace std::placeholders;

        auto handle_goal = [this](const rclcpp_action::GoalUUID & uuid,std::shared_ptr<const MultiespectralAcquisition::Goal> goal)
        {
            logger_->info_stream() << "Received goal request with store flag as: " << (goal->store ? "true" : "false");
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        };

        auto handle_cancel = [this](const std::shared_ptr<MAGoalHandler> goal_handle)
        {
            logger_->info_stream() << "Received request to cancel goal";
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        };


        auto handle_accepted = [this](const std::shared_ptr<MAGoalHandler> goal_handle) 
        {
            std::lock_guard<std::mutex> lock(goal_mutex_);
            if (active_goal_ && (active_goal_->is_active() || active_goal_->is_canceling())) {
                active_goal_->canceled(std::make_shared<MultiespectralAcquisition::Result>());
                logger_->info_stream() << "Previous goal canceled due to new goal arrival.";
            }
            active_goal_ = goal_handle;
            auto execute_in_thread = [this, goal_handle]() { this->execute(goal_handle); };
            std::thread{execute_in_thread}.detach();
        };

        this->action_server_ = rclcpp_action::create_server<MultiespectralAcquisition>(
            this,
            "AS",
            handle_goal,
            handle_cancel,
            handle_accepted);       

        slave_camera_client_ = this->create_client<ImageRequest>("multiespectral_slave_service");

    }

    void execute(const std::shared_ptr<MAGoalHandler> goal_handle)
    { 
        auto goal = goal_handle->get_goal();
        auto action_feedback = std::make_shared<MultiespectralAcquisition::Feedback>();
        auto action_result = std::make_shared<MultiespectralAcquisition::Result>();

        action_feedback->images_acquired = 0;
        action_feedback->storage_path = "none";
        logger_->info_stream() << "[MAMaster::execute] Start image acquisition loop. " << std::string(goal->store?"S":"Not s") << "toring images. Frame rate is "<<std::to_string(this->frame_rate) << "Hz for camera " << getName() << ".";

        if (goal->store)
        {
            action_feedback->storage_path = img_path;
            logger_->info_stream() << "[MAMaster::execute] Storing images to " << img_path;
        }

        bool result = true;
        
        rclcpp::Rate loop_rate(this->frame_rate);
        while (rclcpp::ok())
        {
            if (goal_handle->is_canceling()) 
            {
                goal_handle->canceled(action_result);
                logger_->info_stream() << "Goal canceled.";
                return;
            }
            if (!goal_handle->is_active()) {
                logger_->info_stream() << "Goal is not active anymore, stopping execution.";
                return;
            }

            cv::Mat curr_image(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));  // Init given pattern to check
            createTestPattern(curr_image);

            ImageMetadata metadata;
            metadata.setROSTimeNowCallback([this]() { return this->get_clock()->now().nanoseconds(); });
            logger_->debug_stream() << "[MAMaster::executeCB] Grabbing image.";
            result = this->grabPublishImage(curr_image, metadata);
            if(result && goal->store)
            {
                logger_->debug_stream() << "[MAMaster::executeCB] Storing image.";
                result = this->storeImage(curr_image, metadata);
            }
            if (result) 
            {
                logger_->debug_stream() << "[MAMaster::executeCB] Update action feedback.";                
                if (!curr_image.empty())
                {
                    action_feedback->images_acquired++;
                    action_result->images_acquired = action_feedback->images_acquired;
                }
                goal_handle->publish_feedback(action_feedback);
            }

            logger_->debug_stream() << "[MAMaster::executeCB] Send slave request.";
            auto request = std::make_shared<ImageRequest::Request>();
            request->timestamp = metadata.getSyncTimestamp();
            request->store = goal->store;
            request->visible_pair = metadata.img_name;
            
            if (!slave_camera_client_->wait_for_service(std::chrono::seconds(2))) {
                logger_->error_stream() << "Slave camera service not available";
                goal_handle->abort(action_result);
                return;
            }

            auto future_result = slave_camera_client_->async_send_request(request);
            // logger_->info_stream() << "[MAMaster::executeCB] Prepare request for image with timestamp: " << timestamp;
            // if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result) != rclcpp::FutureReturnCode::SUCCESS) {
            //     logger_->error_stream() << "Slave camera service call failed";
            //     goal_handle->abort(action_result);
            //     return;
            // }

            // auto service_result = future_result.get();
            // if (!service_result->success) {
            //     logger_->error_stream() << "Slave camera service returned failure";
            //     goal_handle->abort(action_result);
            //     return;
            // }

            loop_rate.sleep();
        }
        goal_handle->succeed(action_result);

    }

}; // End class MultiespectralAcquire



int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    std::cout << "[multiespectral_fb_node_master] Starting Multiespectral Acquire Master Node for "<<getType()<<" images." << std::endl;
    auto node = std::make_shared<MultiespectralAcquire>("MultiespectralAcquire_Master_" + getType());
    rclcpp::spin(node);
    rclcpp::shutdown();
}

