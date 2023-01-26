// Copyright 2021 A Team
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

#ifndef ATEAM_COMMON__LOGGER_HPP_
#define ATEAM_COMMON__LOGGER_HPP_

#include <optional>
#include <string>
#include <utility>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

/**
 * @brief Sets up the global logger to redirect all logged output through ROS.
 *
 * Recommended use:
 * Call this macro in the constructor of your node.
 * @code{.cpp}
 * SET_ROS_LOG_HANDLER("my_node");
 * @endcode
 */
#define SET_ROS_LOG_HANDLER(logger_name) \
  ateam_common::Logger::GetLogger().SetRosLogHandler(logger_name);

/**
 * @brief Logs INFO/WARN/ERROR/FATAL using the standard printf style format + args setup. The filename and line number are automatically added as a prefix.
 */
#define LOG_INFO(format, ...) \
  _LOG_LINE(ateam_common::Logger::Level::INFO, format, __VA_ARGS__);
#define LOG_WARN(format, ...) \
  _LOG_LINE(ateam_common::Logger::Level::WARN, format, __VA_ARGS__);
#define LOG_ERROR(format, ...) \
  _LOG_LINE(ateam_common::Logger::Level::ERROR, format, __VA_ARGS__);
#define LOG_FATAL(format, ...) \
  _LOG_LINE(ateam_common::Logger::Level::FATAL, format, __VA_ARGS__);

#define _LOG_LINE(level, format, ...) \
  ateam_common::Logger::GetLogger().Log( \
    level, std::string("[%s:%d] ") + std::string( \
      format), __FILE__, __LINE__, __VA_ARGS__);

namespace ateam_common
{
class Logger
{
public:
  enum Level
  {
    INFO,
    WARN,
    ERROR,
    FATAL
  };

  static Logger & GetLogger()
  {
    static Logger l;
    return l;
  }

  void SetRosLogHandler(const std::string & logger_name)
  {
    ros_logger = rclcpp::get_logger(logger_name);
  }

  template<class ... Args>
  void Log(const Level & level, const std::string & format, Args && ... args)
  {
    if (ros_logger.has_value()) {
      switch (level) {
        case Level::INFO:
          RCLCPP_INFO(ros_logger.value(), format.c_str(), std::forward<Args>(args)...);
          break;
        case Level::WARN:
          RCLCPP_WARN(ros_logger.value(), format.c_str(), std::forward<Args>(args)...);
          break;
        case Level::ERROR:
          RCLCPP_ERROR(ros_logger.value(), format.c_str(), std::forward<Args>(args)...);
          break;
        case Level::FATAL:
          RCLCPP_FATAL(ros_logger.value(), format.c_str(), std::forward<Args>(args)...);
          break;
      }
    } else {
      // Support logging in non-ros contexts (like unit testing)
      printf(format.c_str(), std::forward<Args>(args)...);
      printf("\n\r");
    }
  }

private:
  Logger() = default;

  std::optional<rclcpp::Logger> ros_logger;
};
}  // namespace ateam_common

#endif  // ATEAM_COMMON__LOGGER_HPP_
