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

#include "camera_drivers/camera_adapter.h"

std::string IMAGE_PATH; 
std::string IMAGE_TOPIC;
std::string CAMERA_IP;

// class MultiespectralAcquire;
// std::shared_ptr<MultiespectralAcquire> camera_handler_ptr;
using MultiespectralAcquisition = multiespectral_acquire::action::MultiespectralAcquisition;
using GoalHandle = rclcpp_action::ServerGoalHandle<MultiespectralAcquisition>;
using ImageRequest = multiespectral_acquire::srv::ImageRequest;
using MAGoalHandler = rclcpp_action::ServerGoalHandle<MultiespectralAcquisition>;

class MultiespectralAcquire : public MultiespectralAcquireT
{
protected:
    
    rclcpp_action::Server<MultiespectralAcquisition>::SharedPtr action_server_; 
    std::string action_name_;

    rclcpp::Client<ImageRequest>::SharedPtr slave_camera_client_;
public:
    
    MultiespectralAcquire(std::string name): MultiespectralAcquireT(name), action_name_(name)
    {
        this->init(this->getFrameRate());
        
        using namespace std::placeholders;

        auto handle_goal = [this](const rclcpp_action::GoalUUID & uuid,std::shared_ptr<const MultiespectralAcquisition::Goal> goal)
            {
                RCLCPP_INFO(this->get_logger(), "Received goal request with store flag as: %s", goal->store ? "true" : "false");
                (void)uuid;
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            };

        auto handle_cancel = [this](const std::shared_ptr<MAGoalHandler> goal_handle)
            {
                RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
                (void)goal_handle;
                return rclcpp_action::CancelResponse::ACCEPT;
            };

        auto handle_accepted = [this](const std::shared_ptr<MAGoalHandler> goal_handle)
            {
                // this needs to return quickly to avoid blocking the executor,
                // so we declare a lambda function to be called inside a new thread
                auto execute_in_thread = [this, goal_handle](){return this->execute(goal_handle);};
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

    bool init(int frame_rate)
    {
        this->frame_rate = frame_rate;
        bool result = MultiespectralAcquireT::init(frame_rate);
        // result = result && setAsMaster();

        if(!result) RCLCPP_FATAL_STREAM(get_logger(),"[MAMaster::init] Could not configure " << getName() << " camera as master.");
        if(result) RCLCPP_INFO_STREAM(get_logger(),"[MAMaster::init] Initialized " << getName() << " camera as master with "<<this->frame_rate<<"Hz frame rate.");
        
        result = result && beginAcquisition();
        
        if(result) RCLCPP_INFO_STREAM(get_logger(),SUCCEED_F << "[MAMaster::init] Start image acquisition loop for camera "  << getName() << "." << RESET_F);
        if(!result) 
        {
            RCLCPP_FATAL_STREAM(get_logger(),"[MAMaster::init] Camera init image acquisition failed");
            RCLCPP_FATAL_STREAM(get_logger(),"[MAMaster::MAMaster] Camera init failed");
            throw std::runtime_error("[MAMaster::MAMaster] Camera init failed");
        }
        RCLCPP_INFO_STREAM(get_logger(), SUCCEED_F << "[MAMaster::MAMaster] Camera "<<getName()<<" ("<<getType()<<") initialized successfully" << RESET_F);

        return result;
    }

    void execute(const std::shared_ptr<MAGoalHandler> goal_handle)
    { 
        auto goal = goal_handle->get_goal();
        auto action_feedback = std::make_shared<MultiespectralAcquisition::Feedback>();
        auto action_result = std::make_shared<MultiespectralAcquisition::Result>();

        action_feedback->images_acquired = 0;
        action_feedback->storage_path = "";
        RCLCPP_INFO_STREAM(get_logger(),"[MAMaster::execute] Start image acquisition loop. " << std::string(goal->store?"S":"Not s") << "toring images. Frame rate is "<<std::to_string(this->frame_rate) << "Hz for camera " << getName() << ".");

        if (goal->store)
        {
            action_feedback->storage_path = img_path;
            RCLCPP_INFO_STREAM(get_logger(),"[MAMaster::execute] Storing images to " << img_path);
        }

        bool result = true;
        
        rclcpp::Rate loop_rate(this->frame_rate);
        while (rclcpp::ok())
        {
            if (goal_handle->is_canceling()) {
                goal_handle->canceled(action_result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }

            cv::Mat curr_image(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));  // Init given pattern to check
            createTestPattern(curr_image);

            ImageMetadata metadata;
            RCLCPP_DEBUG(get_logger(), "[MAMaster::executeCB] Grabbing image.");
            result = this->grabPublishImage(curr_image, metadata);
            if(result && goal->store)
            {
                RCLCPP_DEBUG(get_logger(), "[MAMaster::executeCB] Storing image.");
                result = this->storeImage(curr_image, metadata);
            }
            if (result) 
            {
                RCLCPP_DEBUG(get_logger(), "[MAMaster::executeCB] Update action feedback.");
                action_feedback->images_acquired = action_feedback->images_acquired + 1;
                
                if (!curr_image.empty())
                {
                    action_feedback->images_acquired = action_feedback->images_acquired + 1;
                    action_result->images_acquired = action_feedback->images_acquired;
                }
                goal_handle->publish_feedback(action_feedback);
            }

            // RCLCPP_INFO(get_logger(), "[MAMaster::executeCB] Prepare request for image with timestamp: %lu", timestamp);
            RCLCPP_DEBUG(get_logger(), "[MAMaster::executeCB] Send slave request.");
            auto request = std::make_shared<ImageRequest::Request>();
            request->timestamp = metadata.getTimestamp();
            request->store = goal->store;
            request->visible_pair = metadata.img_name;
            
            if (!slave_camera_client_->wait_for_service(std::chrono::seconds(2))) {
                RCLCPP_ERROR_STREAM(get_logger(), "Slave camera service not available");
                goal_handle->abort(action_result);
                return;
            }

            auto future_result = slave_camera_client_->async_send_request(request);
            // RCLCPP_INFO(get_logger(), "[MAMaster::executeCB] Sent request request and spin until completed.");
            // if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result) != rclcpp::FutureReturnCode::SUCCESS) {
            //     RCLCPP_ERROR(get_logger(), "Slave camera service call failed");
            //     goal_handle->abort(action_result);
            //     return;
            // }

            // auto service_result = future_result.get();
            // if (!service_result->success) {
            //     RCLCPP_ERROR(get_logger(), "Slave camera service returned failure");
            //     goal_handle->abort(action_result);
            //     return;
            // }

            loop_rate.sleep();
        }
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

