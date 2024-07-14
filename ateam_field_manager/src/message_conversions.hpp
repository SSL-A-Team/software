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

#include <optional>
#include <string>
#include <vector>
#include <ateam_msgs/msg/field_info.hpp>
#include <ateam_msgs/msg/field_sided_info.hpp>
#include <ateam_common/game_controller_listener.hpp>
#include <ssl_league_msgs/msg/vision_detection_ball.hpp>
#include <ssl_league_msgs/msg/vision_detection_robot.hpp>
#include <ssl_league_msgs/msg/vision_detection_frame.hpp>
#include <ssl_league_msgs/msg/vision_wrapper.hpp>

namespace ateam_field_manager::message_conversions
{

ateam_msgs::msg::FieldInfo fromMsg(
  const ssl_league_msgs::msg::VisionGeometryData & ros_msg,
  const ateam_common::TeamSide & team_side,
  const int ignore_side);
void invertFieldInfo(ateam_msgs::msg::FieldInfo & info);
std::vector<geometry_msgs::msg::Point32> getPointsFromLines(
  const std::vector<ssl_league_msgs::msg::VisionFieldLineSegment> & lines,
  const std::vector<std::string> & line_names);

}  // namespace ateam_field_manager::message_conversions

#endif  // MESSAGE_CONVERSIONS_HPP_
