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

#include "nan_helpers.hpp"
#include <cmath>
#include <rclcpp/logging.hpp>

namespace ateam_radio_bridge
{

void ReplaceNanWithZero(double & val, rclcpp::Logger logger)
{
  if (std::isnan(val)) {
    RCLCPP_WARN(logger, "Radio bridge is replacing NaNs!");
    val = 0.0;
  }
}

void ReplaceNanWithZero(float & val, rclcpp::Logger logger)
{
  if (std::isnan(val)) {
    RCLCPP_WARN(logger, "Radio bridge is replacing NaNs!");
    val = 0.0;
  }
}

void ReplaceNanWithZero(ateam_msgs::msg::Twist2D & twist, rclcpp::Logger logger)
{
  ReplaceNanWithZero(twist.x, logger);
  ReplaceNanWithZero(twist.y, logger);
  ReplaceNanWithZero(twist.theta, logger);
}

void ReplaceNanWithZero(ateam_msgs::msg::RobotMotionCommand & command, rclcpp::Logger logger)
{
  ReplaceNanWithZero(command.pose, logger);
  ReplaceNanWithZero(command.velocity, logger);
  ReplaceNanWithZero(command.acceleration, logger);
  ReplaceNanWithZero(command.limit_vel_linear, logger);
  ReplaceNanWithZero(command.limit_vel_angular, logger);
  ReplaceNanWithZero(command.limit_acc_linear, logger);
  ReplaceNanWithZero(command.limit_acc_angular, logger);
  ReplaceNanWithZero(command.kick_speed, logger);
  ReplaceNanWithZero(command.dribbler_speed, logger);
}

void ReplaceNanWithZero(ateam_msgs::msg::VisionStateRobot & vision_state, rclcpp::Logger logger)
{
  ReplaceNanWithZero(vision_state.pose, logger);
  ReplaceNanWithZero(vision_state.twist, logger);
  ReplaceNanWithZero(vision_state.twist_body, logger);
  ReplaceNanWithZero(vision_state.accel, logger);
}

void ReplaceNanWithZero(geometry_msgs::msg::Pose & pose, rclcpp::Logger logger)
{
  ReplaceNanWithZero(pose.position.x, logger);
  ReplaceNanWithZero(pose.position.y, logger);
  ReplaceNanWithZero(pose.position.z, logger);
  ReplaceNanWithZero(pose.orientation.x, logger);
  ReplaceNanWithZero(pose.orientation.y, logger);
  ReplaceNanWithZero(pose.orientation.z, logger);
  ReplaceNanWithZero(pose.orientation.w, logger);
}

void ReplaceNanWithZero(geometry_msgs::msg::Twist & twist, rclcpp::Logger logger)
{
  ReplaceNanWithZero(twist.linear.x, logger);
  ReplaceNanWithZero(twist.linear.y, logger);
  ReplaceNanWithZero(twist.linear.z, logger);
  ReplaceNanWithZero(twist.angular.x, logger);
  ReplaceNanWithZero(twist.angular.y, logger);
  ReplaceNanWithZero(twist.angular.z, logger);
}

void ReplaceNanWithZero(geometry_msgs::msg::Accel & accel, rclcpp::Logger logger)
{
  ReplaceNanWithZero(accel.linear.x, logger);
  ReplaceNanWithZero(accel.linear.y, logger);
  ReplaceNanWithZero(accel.linear.z, logger);
  ReplaceNanWithZero(accel.angular.x, logger);
  ReplaceNanWithZero(accel.angular.y, logger);
  ReplaceNanWithZero(accel.angular.z, logger);
}

}  // namespace ateam_radio_bridge
