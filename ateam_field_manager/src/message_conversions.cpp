// Copyright 2024 A Team
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

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

namespace ateam_field_manager::message_conversions
{

ateam_msgs::msg::FieldInfo fromMsg(
  const ssl_league_msgs::msg::VisionGeometryData & vision_wrapper_msg,
  const ateam_common::TeamSide & team_side,
  const int ignore_side)
{
  const ssl_league_msgs::msg::VisionGeometryFieldSize & ros_msg =
    vision_wrapper_msg.field;

  ateam_msgs::msg::FieldInfo field_info;

  field_info.field_length = ros_msg.field_length;
  field_info.field_width = ros_msg.field_width;
  field_info.goal_width = ros_msg.goal_width;
  field_info.goal_depth = ros_msg.goal_depth;
  field_info.boundary_width = ros_msg.boundary_width;
  field_info.defense_area_width = ros_msg.penalty_area_width;
  field_info.defense_area_depth = ros_msg.penalty_area_depth;

  field_info.field_corners.points = getPointsFromLines(
    ros_msg.field_lines,
    {"TopTouchLine", "BottomTouchLine"});

  auto Point32Builder = [] {return geometry_msgs::build<geometry_msgs::msg::Point32>();};

  ateam_msgs::msg::FieldSidedInfo left_side_info;

  left_side_info.goal_corners.points = {
    Point32Builder()
    .x(-field_info.field_length / 2)
    .y(field_info.goal_width / 2)
    .z(0),
    Point32Builder()
    .x(-field_info.field_length / 2)
    .y(-field_info.goal_width / 2)
    .z(0),
    Point32Builder()
    .x((-field_info.field_length / 2) - field_info.goal_depth)
    .y(-field_info.goal_width / 2)
    .z(0),
    Point32Builder()
    .x((-field_info.field_length / 2) - field_info.goal_depth)
    .y(field_info.goal_width / 2)
    .z(0)
  };

  left_side_info.defense_area_corners.points = getPointsFromLines(
    ros_msg.field_lines,
    {"LeftFieldLeftPenaltyStretch", "LeftFieldRightPenaltyStretch"});

  // If defense area depth is not set from proto, calculate it from points
  if (field_info.defense_area_depth == 0.0) {
    float max_x = -std::numeric_limits<float>::infinity();
    float min_x = std::numeric_limits<float>::infinity();
    for (const auto & point : left_side_info.defense_area_corners.points) {
      max_x = std::max(max_x, point.x);
      min_x = std::min(min_x, point.x);
    }
    field_info.defense_area_depth = max_x - min_x;
  }

  // If defense area width is not set from proto, calculate it from points
  if (field_info.defense_area_width == 0.0) {
    float max_y = -std::numeric_limits<float>::infinity();
    float min_y = std::numeric_limits<float>::infinity();
    for (const auto & point : left_side_info.defense_area_corners.points) {
      max_y = std::max(max_y, point.y);
      min_y = std::min(min_y, point.y);
    }
    field_info.defense_area_width = max_y - min_y;
  }

  ateam_msgs::msg::FieldSidedInfo right_side_info;
  right_side_info.goal_corners.points = {
    Point32Builder()
    .x(field_info.field_length / 2)
    .y(field_info.goal_width / 2)
    .z(0),
    Point32Builder()
    .x(field_info.field_length / 2)
    .y(-field_info.goal_width / 2)
    .z(0),
    Point32Builder()
    .x((field_info.field_length / 2) + field_info.goal_depth)
    .y(-field_info.goal_width / 2)
    .z(0),
    Point32Builder()
    .x((field_info.field_length / 2) + field_info.goal_depth)
    .y(field_info.goal_width / 2)
    .z(0),
  };

  right_side_info.defense_area_corners.points = getPointsFromLines(
    ros_msg.field_lines,
    {"RightFieldLeftPenaltyStretch", "RightFieldRightPenaltyStretch"});

  const auto circle_iter = std::ranges::find_if(
    ros_msg.field_arcs, [](const auto & line) {
      return line.name == "CenterCircle";
    });

  if (circle_iter != ros_msg.field_arcs.end()) {
    field_info.center_circle = circle_iter->center;
    field_info.center_circle_radius = circle_iter->radius;
  }

  // Assign sides and invert to our coordinate convention if needed.
  switch (team_side) {
    case ateam_common::TeamSide::NegativeHalf:
    case ateam_common::TeamSide::Unknown:
      field_info.ours = left_side_info;
      field_info.theirs = right_side_info;
      break;
    case ateam_common::TeamSide::PositiveHalf:
      field_info.ours = right_side_info;
      field_info.theirs = left_side_info;
      invertFieldInfo(field_info);
      break;
  }

  field_info.ignore_side = ignore_side;

  return field_info;
}

void invertFieldInfo(ateam_msgs::msg::FieldInfo & info)
{
  auto invert_point_array = [&](auto & target_array) {
      for (auto & point : target_array) {
        point.x *= -1.0;
        point.y *= -1.0;
      }
    };

  invert_point_array(info.field_corners.points);
  invert_point_array(info.ours.defense_area_corners.points);
  invert_point_array(info.ours.goal_corners.points);
  invert_point_array(info.theirs.defense_area_corners.points);
  invert_point_array(info.theirs.goal_corners.points);
}

std::vector<geometry_msgs::msg::Point32> getPointsFromLines(
  const std::vector<ssl_league_msgs::msg::VisionFieldLineSegment> & lines,
  const std::vector<std::string> & line_names)
{
  std::vector<geometry_msgs::msg::Point32> points;
  points.reserve(line_names.size() * 2);
  for (const auto & name : line_names) {
    const auto line_iter = std::ranges::find_if(
      lines, [&name](const auto & line) {
        return line.name == name;
      });

    if (line_iter == lines.end()) {
      // TODO(barulicm) Log missing line?
      continue;
    }

    geometry_msgs::msg::Point32 p1;
    p1.x = line_iter->p1.x;
    p1.y = line_iter->p1.y;
    points.push_back(p1);
    geometry_msgs::msg::Point32 p2;
    p2.x = line_iter->p2.x;
    p2.y = line_iter->p2.y;
    points.push_back(p2);
  }
  return points;
}

}  // namespace ateam_field_manager::message_conversions
