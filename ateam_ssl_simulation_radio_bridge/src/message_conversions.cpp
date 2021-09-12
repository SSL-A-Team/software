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

#include "message_conversions.hpp"

namespace ateam_ssl_simulation_radio_bridge::message_conversions
{

void fromMsg(const ateam_msgs::msg::RobotMotionCommand & ros_msg, RobotCommand * proto_msg)
{
  proto_msg->set_id(ros_msg.id);

  RobotMoveCommand * robot_move_command = proto_msg->mutable_move_command();
  MoveGlobalVelocity * global_velocity_command =
    robot_move_command->mutable_global_velocity();
  global_velocity_command->set_x(ros_msg.twist.linear.x);
  global_velocity_command->set_y(ros_msg.twist.linear.y);
  global_velocity_command->set_angular(ros_msg.twist.angular.z);
}

RobotControl fromMsg(const ateam_msgs::msg::RobotCommands & ros_msg)
{
  RobotControl robots_control;

  for (const auto & ros_robot_command : ros_msg.commands) {
    RobotCommand * proto_robot_command = robots_control.add_robot_commands();
    fromMsg(ros_robot_command, proto_robot_command);
  }

  return robots_control;
}

}  // namespace ateam_ssl_simulation_radio_bridge::message_conversions
