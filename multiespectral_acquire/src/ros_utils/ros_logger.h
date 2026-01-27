/**
 * @file    ros_logger.h
 * @brief   Common ROS logger implementation for all nodes
 */

#ifndef ROS_LOGGER_H
#define ROS_LOGGER_H

#include <ros/ros.h>
#include "core/utils/logging_utils.h"

class RosLogger : public Logger {
public:
    RosLogger() {}
    void debug(const std::string& msg) override { ROS_DEBUG("%s", msg.c_str()); }
    void info(const std::string& msg) override { ROS_INFO("%s", msg.c_str()); }
    void warn(const std::string& msg) override { ROS_WARN("%s", msg.c_str()); }
    void error(const std::string& msg) override { ROS_ERROR("%s", msg.c_str()); }
    void fatal(const std::string& msg) override { ROS_FATAL("%s", msg.c_str()); }
};

#endif // ROS_LOGGER_H
