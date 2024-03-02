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

#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <string>
#include <vector>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <ateam_common/node_handle.hpp>

namespace ateam_vision_filter::message_conversions
{

ateam_msgs::msg::BallState toMsg(const std::optional<Ball> & maybe_ball)
{
  ateam_msgs::msg::BallState ball_state_msg;
  ball_state_msg.visible = maybe_ball.has_value();
  if (maybe_ball.has_value()) {
    auto obj = maybe_ball.value();

    ball_state_msg.pose.position.x = obj.position.x();
    ball_state_msg.pose.position.y = obj.position.y();
    ball_state_msg.twist.linear.x = obj.velocity.x();
    ball_state_msg.twist.linear.y = obj.velocity.y();
    ball_state_msg.accel.linear.x = obj.acceleration.x();
    ball_state_msg.accel.linear.y = obj.acceleration.y();
  }

  return ball_state_msg;
}

ateam_msgs::msg::RobotState toMsg(const std::optional<Robot> & maybe_robot)
{
  ateam_msgs::msg::RobotState robot_state_msg;
  robot_state_msg.visible = maybe_robot.has_value();

  if (maybe_robot.has_value()) {
    auto obj = maybe_robot.value();
    robot_state_msg.timestamp = ateam_common::node_handle::now();
    robot_state_msg.pose.position.x = obj.position.x();
    robot_state_msg.pose.position.y = obj.position.y();
    robot_state_msg.twist.linear.x = obj.velocity.x();
    robot_state_msg.twist.linear.y = obj.velocity.y();
    robot_state_msg.accel.linear.x = obj.acceleration.x();
    robot_state_msg.accel.linear.y = obj.acceleration.y();
    robot_state_msg.pose.orientation =
      tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), obj.theta));
    robot_state_msg.twist.angular.z = obj.omega;
    robot_state_msg.accel.angular.z = obj.alpha;
  }

  return robot_state_msg;
}

CameraMeasurement fromMsg(
  const ssl_league_msgs::msg::VisionDetectionFrame & ros_msg,
  const ateam_common::TeamSide & team_side)
{
  CameraMeasurement cameraFrame;
  for (const auto & ball_detection : ros_msg.balls) {
    cameraFrame.ball.push_back(fromMsg(ball_detection));
  }

  for (const auto & yellow_robot_detection : ros_msg.robots_yellow) {
    std::size_t robot_id = yellow_robot_detection.robot_id;
    cameraFrame.yellow_robots.at(robot_id).push_back(fromMsg(yellow_robot_detection));
  }

  for (const auto & blue_robot_detection : ros_msg.robots_blue) {
    std::size_t robot_id = blue_robot_detection.robot_id;
    cameraFrame.blue_robots.at(robot_id).push_back(fromMsg(blue_robot_detection));
  }

  if (team_side == ateam_common::TeamSide::PositiveHalf) {
    cameraFrame.invert();
  }

  return cameraFrame;
}

RobotMeasurement fromMsg(const ssl_league_msgs::msg::VisionDetectionRobot & ros_msg)
{
  tf2::Quaternion tf2_quat;
  RobotMeasurement robotDetection;
  robotDetection.position.x() = ros_msg.pose.position.x;
  robotDetection.position.y() = ros_msg.pose.position.y;
  tf2::fromMsg(ros_msg.pose.orientation, tf2_quat);
  robotDetection.theta = tf2::getYaw(tf2_quat);
  return robotDetection;
}

BallMeasurement fromMsg(const ssl_league_msgs::msg::VisionDetectionBall & ros_msg)
{
  BallMeasurement ballDetection;
  ballDetection.position.x() = ros_msg.pos.x;
  ballDetection.position.y() = ros_msg.pos.y;
  return ballDetection;
}

ateam_msgs::msg::FieldInfo fromMsg(
  const ssl_league_msgs::msg::VisionGeometryData & vision_wrapper_msg,
  const ateam_common::TeamSide & team_side)
{
  const ssl_league_msgs::msg::VisionGeometryFieldSize & ros_msg =
    vision_wrapper_msg.field;

  ateam_msgs::msg::FieldInfo field_info;

  field_info.field_length = ros_msg.field_length;
  field_info.field_width = ros_msg.field_width;
  field_info.goal_width = ros_msg.goal_width;
  field_info.goal_depth = ros_msg.goal_depth;
  field_info.boundary_width = ros_msg.boundary_width;

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

}  // namespace ateam_vision_filter::message_conversions
