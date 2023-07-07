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

ateam_msgs::msg::RobotFeedback fromProto(const RobotFeedback & proto_msg)
{
  ateam_msgs::msg::RobotFeedback robot_feedback;

  robot_feedback.radio_connected = true;
  robot_feedback.breakbeam_ball_detected = proto_msg.dribbler_ball_contact();

  return robot_feedback;
}

RobotControl fromMsg(const ateam_msgs::msg::RobotMotionCommand & ros_msg, int robot_id, const bool ball_in_breakbeam)
{
  RobotControl robots_control;
  RobotCommand * proto_robot_command = robots_control.add_robot_commands();

  proto_robot_command->set_id(robot_id);
  proto_robot_command->set_dribbler_speed(2000);

  switch(ros_msg.kick) {
    case ateam_msgs::msg::RobotMotionCommand::KICK_ARM:
    case ateam_msgs::msg::RobotMotionCommand::KICK_DISABLE:
      // Do nothing because the simulator doesn't "charge" kickers
      break;
    case ateam_msgs::msg::RobotMotionCommand::KICK_NOW:
      proto_robot_command->set_kick_speed(ros_msg.kick_speed);
      break;
    case ateam_msgs::msg::RobotMotionCommand::KICK_ON_TOUCH:
    case ateam_msgs::msg::RobotMotionCommand::KICK_ON_CAPTURE:
      if(ball_in_breakbeam) {
        proto_robot_command->set_kick_speed(ros_msg.kick_speed);
      }
      break;
  }

  RobotMoveCommand * robot_move_command = proto_robot_command->mutable_move_command();
  MoveGlobalVelocity * global_velocity_command =
    robot_move_command->mutable_global_velocity();

  global_velocity_command->set_x(ros_msg.twist.linear.x);
  global_velocity_command->set_y(ros_msg.twist.linear.y);
  global_velocity_command->set_angular(ros_msg.twist.angular.z);

  return robots_control;
}

}  // namespace ateam_ssl_simulation_radio_bridge::message_conversions
