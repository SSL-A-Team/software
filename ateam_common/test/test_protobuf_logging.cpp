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
#include <google/protobuf/stubs/common.h>
#include <string>
#include "ateam_common/protobuf_logging.hpp"

const char * logger_name = "test.protobuf";
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

class TestProtobufLogging : public testing::Test
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

TEST_F(TestProtobufLogging, test_setup_message)
{
  SET_ROS_PROTOBUF_LOG_HANDLER(logger_name);
  EXPECT_EQ(1u, log_calls);
  EXPECT_EQ(RCUTILS_LOG_SEVERITY_INFO, last_log_event.level);
  EXPECT_EQ(logger_name, last_log_event.name);
  EXPECT_EQ("Routing protobuf logs through ROS", last_log_event.message);
}

TEST_F(TestProtobufLogging, test_info)
{
  SET_ROS_PROTOBUF_LOG_HANDLER(logger_name);
  GOOGLE_LOG(INFO) << "Info log";
  EXPECT_EQ(2u, log_calls);
  EXPECT_EQ(RCUTILS_LOG_SEVERITY_INFO, last_log_event.level);
  EXPECT_EQ(logger_name, last_log_event.name);
  EXPECT_THAT(
    last_log_event.message,
    ::testing::MatchesRegex("\\[.*test_protobuf_logging\\.cpp\\:[0-9]+\\] Info log"));
}

TEST_F(TestProtobufLogging, test_warning)
{
  SET_ROS_PROTOBUF_LOG_HANDLER(logger_name);
  GOOGLE_LOG(WARNING) << "Warning log";
  EXPECT_EQ(2u, log_calls);
  EXPECT_EQ(RCUTILS_LOG_SEVERITY_WARN, last_log_event.level);
  EXPECT_EQ(logger_name, last_log_event.name);
  EXPECT_THAT(
    last_log_event.message,
    ::testing::MatchesRegex("\\[.*test_protobuf_logging\\.cpp\\:[0-9]+\\] Warning log"));
}

TEST_F(TestProtobufLogging, test_error)
{
  SET_ROS_PROTOBUF_LOG_HANDLER(logger_name);
  GOOGLE_LOG(ERROR) << "Error log";
  EXPECT_EQ(2u, log_calls);
  EXPECT_EQ(RCUTILS_LOG_SEVERITY_ERROR, last_log_event.level);
  EXPECT_EQ(logger_name, last_log_event.name);
  EXPECT_THAT(
    last_log_event.message,
    ::testing::MatchesRegex("\\[.*test_protobuf_logging\\.cpp\\:[0-9]+\\] Error log"));
}

TEST_F(TestProtobufLogging, test_fatal)
{
  SET_ROS_PROTOBUF_LOG_HANDLER(logger_name);
  try {
    GOOGLE_LOG(FATAL) << "Fatal log";
  } catch (const google::protobuf::FatalException &) {
    // ignore the exception thrown every time fatal errors are logged
  }
  EXPECT_EQ(2u, log_calls);
  EXPECT_EQ(RCUTILS_LOG_SEVERITY_FATAL, last_log_event.level);
  EXPECT_EQ(logger_name, last_log_event.name);
  EXPECT_THAT(
    last_log_event.message,
    ::testing::MatchesRegex("\\[.*test_protobuf_logging\\.cpp\\:[0-9]+\\] Fatal log"));
}
