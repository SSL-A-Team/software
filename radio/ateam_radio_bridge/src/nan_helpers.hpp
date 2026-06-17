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

#ifndef ATEAM_RADIO_BRIDGE__RADIO_BRIDGE_NODE_HPP_
#define ATEAM_RADIO_BRIDGE__RADIO_BRIDGE_NODE_HPP_

#include <ateam_msgs/msg/twist2_d.hpp>
#include <ateam_msgs/msg/robot_motion_command.hpp>
#include <ateam_msgs/msg/vision_state_robot.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <rclcpp/logger.hpp>

#define REPLACE_NAN_WITH_ZERO(val) ateam_radio_bridge::ReplaceNanWithZero(val, get_logger())

namespace ateam_radio_bridge
{

void ReplaceNanWithZero(double & val, rclcpp::Logger logger);

void ReplaceNanWithZero(float & val, rclcpp::Logger logger);

void ReplaceNanWithZero(ateam_msgs::msg::Twist2D & twist, rclcpp::Logger logger);

void ReplaceNanWithZero(ateam_msgs::msg::RobotMotionCommand & command, rclcpp::Logger logger);

void ReplaceNanWithZero(ateam_msgs::msg::VisionStateRobot & vision_state, rclcpp::Logger logger);

void ReplaceNanWithZero(geometry_msgs::msg::Pose & pose, rclcpp::Logger logger);

void ReplaceNanWithZero(geometry_msgs::msg::Twist & twist, rclcpp::Logger logger);

void ReplaceNanWithZero(geometry_msgs::msg::Accel & accel, rclcpp::Logger logger);

}  // namespace ateam_radio_bridge

#endif  // ATEAM_RADIO_BRIDGE__RADIO_BRIDGE_NODE_HPP_
