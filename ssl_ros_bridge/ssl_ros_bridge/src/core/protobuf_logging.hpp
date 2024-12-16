// Copyright 2024 A Team
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef CORE__PROTOBUF_LOGGING_HPP_
#define CORE__PROTOBUF_LOGGING_HPP_

#include <google/protobuf/stubs/logging.h>
#include <string>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

/**
 * @brief Sets up the protobuf log handler to redirect all messages through ROS.
 *
 * Recommended use:
 * Call this macro in the constructor of your node. For the logger name, append ".protobuf" to your
 * node name to create a child of your node's logger.
 * @code{.cpp}
 * SET_ROS_PROTOBUF_LOG_HANDLER("my_node.protobuf");
 * @endcode
 *
 */
#define SET_ROS_PROTOBUF_LOG_HANDLER(logger_name) \
  { \
    google::protobuf::SetLogHandler( \
      [](google::protobuf::LogLevel level, const char * filename, int line, \
      const std::string & message) { \
        auto ros_logger = rclcpp::get_logger(logger_name); \
        switch (level) { \
          case google::protobuf::LogLevel::LOGLEVEL_INFO: \
            RCLCPP_INFO(ros_logger, "[%s:%d] %s", filename, line, message.c_str()); \
            break; \
          case google::protobuf::LogLevel::LOGLEVEL_WARNING: \
            RCLCPP_WARN(ros_logger, "[%s:%d] %s", filename, line, message.c_str()); \
            break; \
          case google::protobuf::LogLevel::LOGLEVEL_ERROR: \
            RCLCPP_ERROR(ros_logger, "[%s:%d] %s", filename, line, message.c_str()); \
            break; \
          case google::protobuf::LogLevel::LOGLEVEL_FATAL: \
            RCLCPP_FATAL(ros_logger, "[%s:%d] %s", filename, line, message.c_str()); \
            break; \
        } \
      }); \
    RCLCPP_INFO(rclcpp::get_logger(logger_name), "Routing protobuf logs through ROS"); \
  }

#endif  // CORE__PROTOBUF_LOGGING_HPP_
