// Copyright 2025 A Team
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

#include "joystick_enforcer.hpp"
#include <string>
#include <ateam_common/topic_names.hpp>

namespace ateam_kenobi
{

JoystickEnforcer::JoystickEnforcer(rclcpp::Node & node)
{
  status_sub_ =
    node.create_subscription<ateam_msgs::msg::JoystickControlStatus>(std::string(
      Topics::kJoystickControlStatus), rclcpp::SystemDefaultsQoS(),
      std::bind(&JoystickEnforcer::JoystickControlStatusCallback, this, std::placeholders::_1));
}

void JoystickEnforcer::RemoveCommandForJoystickBot(
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> & robot_motion_commands)
{
  if(!active_) {
    return;
  }
  robot_motion_commands[robot_id_].reset();
}

void JoystickEnforcer::JoystickControlStatusCallback(
  const ateam_msgs::msg::JoystickControlStatus::ConstSharedPtr msg)
{
  if(msg->active_id < -1 || msg->active_id > 15) {
    active_ = false;
    robot_id_ = -1;
  } else {
    active_ = msg->is_active;
    robot_id_ = msg->active_id;
  }
}

}  // namespace ateam_kenobi
