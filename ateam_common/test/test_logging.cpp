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

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <string>
#include "ateam_common/logger.hpp"

const char * logger_name = "test";
size_t log_calls = 0;
rclcpp::Logger logger = rclcpp::get_logger(logger_name);

struct LogEvent
{
  const rcutils_log_location_t * location;
  int level;
  std::string name;
  rcutils_time_point_value_t timestamp;
  std::string message;
};
LogEvent last_log_event;

class TestLogging : public testing::Test
{
public:
  rcutils_logging_output_handler_t previous_output_handler;

  void SetUp()
  {
    log_calls = 0;
    ASSERT_EQ(RCUTILS_RET_OK, rcutils_logging_initialize());
    rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_DEBUG);

    auto output_handler = [](const rcutils_log_location_t * location,
        int level, const char * name, rcutils_time_point_value_t timestamp,
        const char * format, va_list * args) {
        ++log_calls;
        last_log_event = LogEvent{
          location,
          level,
          (name ? name : ""),
          timestamp,
          ""
        };
        char buffer[1024];
        vsnprintf(buffer, sizeof(buffer), format, *args);
        last_log_event.message = buffer;
      };

    previous_output_handler = rcutils_logging_get_output_handler();
    rcutils_logging_set_output_handler(output_handler);
  }

  void TearDown()
  {
    rcutils_logging_set_output_handler(previous_output_handler);
    ASSERT_EQ(RCUTILS_RET_OK, rcutils_logging_shutdown());
    EXPECT_FALSE(g_rcutils_logging_initialized);
  }
};

TEST_F(TestLogging, test_info)
{
  SET_ROS_LOG_HANDLER(logger_name);
  LOG_INFO("Hello %s", "World");
  EXPECT_EQ(log_calls, 1u);
  EXPECT_EQ(last_log_event.level, RCUTILS_LOG_SEVERITY_INFO);
  EXPECT_EQ(last_log_event.name, logger_name);
  EXPECT_THAT(
    last_log_event.message,
    ::testing::MatchesRegex("\\[.*test_logging\\.cpp\\:[0-9]+\\] Hello World"));
}

TEST_F(TestLogging, test_warn)
{
  SET_ROS_LOG_HANDLER(logger_name);
  LOG_WARN("Hello %s", "World");
  EXPECT_EQ(log_calls, 1u);
  EXPECT_EQ(last_log_event.level, RCUTILS_LOG_SEVERITY_WARN);
  EXPECT_EQ(last_log_event.name, logger_name);
  EXPECT_THAT(
    last_log_event.message,
    ::testing::MatchesRegex("\\[.*test_logging\\.cpp\\:[0-9]+\\] Hello World"));
}

TEST_F(TestLogging, test_error)
{
  SET_ROS_LOG_HANDLER(logger_name);
  LOG_ERROR("Hello %s", "World");
  EXPECT_EQ(log_calls, 1u);
  EXPECT_EQ(last_log_event.level, RCUTILS_LOG_SEVERITY_ERROR);
  EXPECT_EQ(last_log_event.name, logger_name);
  EXPECT_THAT(
    last_log_event.message,
    ::testing::MatchesRegex("\\[.*test_logging\\.cpp\\:[0-9]+\\] Hello World"));
}

TEST_F(TestLogging, test_fatal)
{
  SET_ROS_LOG_HANDLER(logger_name);
  LOG_FATAL("Hello %s", "World");
  EXPECT_EQ(log_calls, 1u);
  EXPECT_EQ(last_log_event.level, RCUTILS_LOG_SEVERITY_FATAL);
  EXPECT_EQ(last_log_event.name, logger_name);
  EXPECT_THAT(
    last_log_event.message,
    ::testing::MatchesRegex("\\[.*test_logging\\.cpp\\:[0-9]+\\] Hello World"));
}
