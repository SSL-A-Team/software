// Copyright 2023 A Team
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

#include "constant_vel_test.hpp"
#include <functional>
#include <ateam_common/topic_names.hpp>
#include "play_helpers/available_robots.hpp"
#include "singleton_node.hpp"

namespace ateam_kenobi::plays
{

ConstantVelTest::ConstantVelTest(
  visualization::OverlayPublisher & overlay_publisher,
  visualization::PlayInfoPublisher & play_info_publisher)
: BasePlay(overlay_publisher, play_info_publisher)
{
  ateam_kenobi::global_node->declare_parameters<double>("constant_vel_test.vel",{
    {"x", 0.0},
    {"y", 1.0},
    {"t", 0.0}
  });
  joy_sub = ateam_kenobi::global_node->create_subscription<sensor_msgs::msg::Joy>(std::string(Topics::kJoystick), rclcpp::QoS{1}, std::bind(&ConstantVelTest::joystickCallback, this, std::placeholders::_1));
  // set_parameters_callback_handle = ateam_kenobi::global_node->add_on_set_parameters_callback(std::bind_front(&ConstantVelTest::onSetParametersCallback, this));
}

void ConstantVelTest::reset()
{

}

std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> ConstantVelTest::runFrame(const World & world)
{
  const auto current_available_robots = play_helpers::getAvailableRobots(world);
  if(current_available_robots.empty()) {
    return {};
  }

  const auto & robot = current_available_robots.front();

  ateam_msgs::msg::RobotMotionCommand motion_command;
  if(motion_enabled) {
    motion_command.twist.linear.x = ateam_kenobi::global_node->get_parameter("constant_vel_test.vel.x").get_value<double>();
    motion_command.twist.linear.y = ateam_kenobi::global_node->get_parameter("constant_vel_test.vel.y").get_value<double>();
    motion_command.twist.angular.z = ateam_kenobi::global_node->get_parameter("constant_vel_test.vel.t").get_value<double>();
  } else {
    motion_command.twist.linear.x = 0.0;
    motion_command.twist.linear.y = 0.0;
    motion_command.twist.angular.z = 0.0;
  }

  play_info_publisher_.message["velocity"]["x"] = motion_command.twist.linear.x;
  play_info_publisher_.message["velocity"]["y"] = motion_command.twist.linear.y;
  play_info_publisher_.message["velocity"]["z"] = motion_command.twist.angular.z;
  play_info_publisher_.message["enabled"] = (motion_enabled ? "true" : "false");
  play_info_publisher_.send_play_message("Constant Vel");

  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> maybe_motion_commands;
  maybe_motion_commands[robot.id] = motion_command;
  return maybe_motion_commands;
}

void ConstantVelTest::joystickCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  motion_enabled = msg->buttons[0];
}

rcl_interfaces::msg::SetParametersResult ConstantVelTest::onSetParametersCallback(const std::vector<rclcpp::Parameter> &)
{
  vel_x = ateam_kenobi::global_node->get_parameter("constant_vel_test.vel.x").get_value<double>();
  vel_y = ateam_kenobi::global_node->get_parameter("constant_vel_test.vel.y").get_value<double>();
  vel_t = ateam_kenobi::global_node->get_parameter("constant_vel_test.vel.t").get_value<double>();
  
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;

}

}  // namespace ateam_kenobi::plays
