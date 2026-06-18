/**
 * @file    ros_logger.h
 * @brief   ROS2 logger implementation wrapping rclcpp logger into the Logger interface
 */

#ifndef ROS_LOGGER_H
#define ROS_LOGGER_H

#include <rclcpp/rclcpp.hpp>
#include "core/utils/logging_utils.h"

class RosLogger : public Logger {
    rclcpp::Logger logger_;
public:
    explicit RosLogger(const rclcpp::Logger& logger) : logger_(logger) {}

    void debug(const std::string& msg) override { RCLCPP_DEBUG(logger_, "%s", msg.c_str()); }
    void info (const std::string& msg) override { RCLCPP_INFO (logger_, "%s", msg.c_str()); }
    void warn (const std::string& msg) override { RCLCPP_WARN (logger_, "%s", msg.c_str()); }
    void error(const std::string& msg) override { RCLCPP_ERROR(logger_, "%s", msg.c_str()); }
    void fatal(const std::string& msg) override { RCLCPP_FATAL(logger_, "%s", msg.c_str()); }
};

#endif // ROS_LOGGER_H
