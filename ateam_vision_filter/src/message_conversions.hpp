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

#include <ateam_msgs/msg/ball_state.hpp>
#include <ateam_msgs/msg/robot_state.hpp>
#include <ateam_msgs/msg/field_info.hpp>
#include <ateam_msgs/msg/field_sided_info.hpp>
#include <ssl_league_msgs/msg/vision_detection_ball.hpp>
#include <ssl_league_msgs/msg/vision_detection_robot.hpp>
#include <ssl_league_msgs/msg/vision_detection_frame.hpp>
#include <ssl_league_msgs/msg/vision_wrapper.hpp>
#include <optional>

#include "types/ball.hpp"
#include "types/ball_measurement.hpp"
#include "types/robot.hpp"
#include "types/robot_measurement.hpp"
#include "types/camera_measurement.hpp"

namespace ateam_vision_filter::message_conversions
{

ateam_msgs::msg::BallState toMsg(const std::optional<Ball> & maybe_ball);
ateam_msgs::msg::RobotState toMsg(const std::optional<Robot> & maybe_robot);

CameraMeasurement getCameraMeasurement(const ssl_league_msgs::msg::VisionWrapper & ros_msg);
CameraMeasurement fromMsg(const ssl_league_msgs::msg::VisionDetectionFrame & ros_msg);
RobotMeasurement fromMsg(const ssl_league_msgs::msg::VisionDetectionRobot & ros_msg);
BallMeasurement fromMsg(const ssl_league_msgs::msg::VisionDetectionBall & ros_msg);

void invert_field_info(ateam_msgs::msg::FieldInfo & info);

ateam_msgs::msg::FieldInfo getFieldGeometry(const ssl_league_msgs::msg::VisionWrapper & ros_msg);
ateam_msgs::msg::FieldInfo fromMsg(const ssl_league_msgs::msg::VisionGeometryData & ros_msg);

}  // namespace ateam_vision_filter::message_conversions

#endif  // MESSAGE_CONVERSIONS_HPP_
