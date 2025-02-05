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

#ifndef JOYSTICK_ENFORCER_HPP__
#define JOYSTICK_ENFORCER_HPP__

#include <rclcpp/rclcpp.hpp>
#include <ateam_msgs/msg/joystick_control_status.hpp>
#include <ateam_msgs/msg/robot_motion_command.hpp>

namespace ateam_kenobi
{

class JoystickEnforcer {
public:
  JoystickEnforcer(rclcpp::Node & node);

  void RemoveCommandForJoystickBot(std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> & robot_motion_commands);

private:
  rclcpp::Subscription<ateam_msgs::msg::JoystickControlStatus>::SharedPtr status_sub_;
  bool active_ = false;
  int robot_id_ = -1;

  void JoystickControlStatusCallback(const ateam_msgs::msg::JoystickControlStatus::ConstSharedPtr msg);

};

}  // namespace ateam_kenobi

#endif  // JOYSTICK_ENFORCER_HPP__
