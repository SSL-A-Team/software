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

#ifndef MESSAGE_CONVERSIONS_HPP_
#define MESSAGE_CONVERSIONS_HPP_

#include <rclcpp/logger.hpp>

#include <ssl_league_protobufs/ssl_simulation_robot_control.pb.h>
#include <ssl_league_protobufs/ssl_simulation_robot_feedback.pb.h>
#include <ssl_league_protobufs/ssl_simulation_control.pb.h>

#include <rclcpp/logger.hpp>

#include <ssl_league_msgs/msg/simulator_control.hpp>

#include <ateam_radio_msgs/msg/basic_telemetry.hpp>
#include <ateam_msgs/msg/robot_motion_command.hpp>

namespace ateam_ssl_simulation_radio_bridge::message_conversions
{

ateam_radio_msgs::msg::BasicTelemetry fromProto(const RobotFeedback & proto_msg);

RobotControl fromMsg(
  const ateam_msgs::msg::RobotMotionCommand & ros_msg, int robot_id,
  rclcpp::Logger logger);

SimulatorControl fromMsg(const ssl_league_msgs::msg::SimulatorControl & ros_msg);

}  // namespace ateam_ssl_simulation_radio_bridge::message_conversions

#endif  // MESSAGE_CONVERSIONS_HPP_
