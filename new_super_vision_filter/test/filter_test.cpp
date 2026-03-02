// Copyright 2026 A Team
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

#include <gtest/gtest.h>
#include <chrono>
#include <thread>
#include <memory>
#include <ostream>
#include <iostream>

#include <ssl_league_msgs/msg/vision_detection_robot.hpp>
#include <ateam_msgs/msg/vision_state_robot.hpp>
#include <ateam_common/game_controller_listener.hpp>

#include "filtered_robot.hpp"
#include "measurements/robot_track.hpp"

namespace ateam_msgs
{
namespace msg
{
void PrintTo(const VisionStateRobot & state, std::ostream * os)
{
  *os << "\n" << "VisionStateRobot: \n" <<
    "x pos: " << state.pose.position.x << " y pos: " << state.pose.position.y <<
    " z pos: " << state.pose.position.z << "\n" << "visible? " << state.visible <<
    "\n" << "orientation: " << state.pose.orientation.w;
}
}
}

class FilteredRobotTest : public ::testing::Test
{
protected:
  ateam_common::TeamColor team = ateam_common::TeamColor::Blue;
  int oldEnoughAge = 2;
  ssl_league_msgs::msg::VisionDetectionRobot robot_msg{};
  std::unique_ptr<FilteredRobot> bot;

  void SetUp() override
  {
    int camera = 0;
    auto track = RobotTrack(robot_msg, camera, team);
    bot = std::make_unique<FilteredRobot>(track, team);
  }
};

TEST_F(FilteredRobotTest, WaitUntilOldEnough)
{
  ateam_msgs::msg::VisionStateRobot default_msg{};
  int camera = 0;
  for (size_t i = 0; i < oldEnoughAge; ++i) {
      // Set it up so we can print our messages more effectively
      // https://google.github.io/googletest/advanced.html#teaching-googletest-how-to-print-your-values
    ssl_league_msgs::msg::VisionDetectionRobot fake_vision_data{};
    fake_vision_data.pose.position.x = 1;
    fake_vision_data.pose.position.y = 1;
    if (i < oldEnoughAge) {
      auto fake_track = RobotTrack(fake_vision_data, camera, team);
      bot->update(fake_track);
      auto msg = bot->toMsg();
        // We shouldn't update if our filter is too new
      EXPECT_EQ(default_msg, msg);
    } else {
        // Sorry, adding a short sleep was easier than mocking the timestamp...
      std::this_thread::sleep_for(std::chrono::milliseconds(60));
      auto fake_track = RobotTrack(fake_vision_data, camera, team);
      bot->update(fake_track);
      auto msg = bot->toMsg();
        // We should update if our filter is old enough
      EXPECT_NE(default_msg, msg);
    }
    }
}
