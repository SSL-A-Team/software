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

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace ateam_vision_filter::message_conversions
{

ateam_msgs::msg::BallState toMsg(const Ball & obj)
{
  ateam_msgs::msg::BallState ball_state_msg;
  ball_state_msg.pose.position.x = obj.position.x();
  ball_state_msg.pose.position.y = obj.position.y();
  ball_state_msg.twist.linear.x = obj.velocity.x();
  ball_state_msg.twist.linear.y = obj.velocity.y();
  ball_state_msg.accel.linear.x = obj.acceleration.x();
  ball_state_msg.accel.linear.y = obj.acceleration.y();

  return ball_state_msg;
}

ateam_msgs::msg::RobotState toMsg(const Robot & obj)
{
  ateam_msgs::msg::RobotState robot_state_msg;
  robot_state_msg.pose.position.x = obj.position.x();
  robot_state_msg.pose.position.y = obj.position.y();
  robot_state_msg.twist.linear.x = obj.velocity.x();
  robot_state_msg.twist.linear.y = obj.velocity.y();
  robot_state_msg.accel.linear.x = obj.acceleration.x();
  robot_state_msg.accel.linear.y = obj.acceleration.y();
  robot_state_msg.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), obj.theta));
  robot_state_msg.twist.angular.z = obj.omega;
  robot_state_msg.accel.angular.z = obj.alpha;

  return robot_state_msg;
}

CameraMeasurement getCameraMeasurement(const ssl_league_msgs::msg::VisionWrapper & ros_msg)
{
  return fromMsg(ros_msg.detection);
}

ateam_msgs::msg::FieldInfo getFieldGeometry(const ssl_league_msgs::msg::VisionWrapper & ros_msg)
{
  return fromMsg(ros_msg.geometry);
}

// note left and right can be different according to Joe
ateam_msgs::msg::FieldInfo fromMsg(
  const ssl_league_msgs::msg::VisionGeometryData & vision_wrapper_msg)
{
  // calibration left out but should be included in field info message if needed
  const ssl_league_msgs::msg::VisionGeometryFieldSize & ros_msg =
    vision_wrapper_msg.field;

  ateam_msgs::msg::FieldInfo field_info {};
  // check for if invalid since we cant see the original optional and this
  // is a rough estimate of the smallest field
  if (ros_msg.field_length < 1.0 || ros_msg.field_width < 1.0) {
    field_info.valid = false;
    return field_info;
  }

  field_info.field_length = ros_msg.field_length;
  field_info.field_width = ros_msg.field_width;
  field_info.goal_width = ros_msg.goal_width;
  field_info.goal_depth = ros_msg.goal_depth;
  field_info.boundary_width = ros_msg.boundary_width;

  auto check_field_line_name =
    [](ssl_league_msgs::msg::VisionFieldLineSegment line_msg, std::string target_name) -> bool {
      return line_msg.name == target_name;
    };


  // did just realize I could have done this as a std transform
  auto lines_to_points = [&](auto & name_array, auto & target_array) {
      target_array.resize(name_array.size() * 2);
      for (size_t i = 0; i < name_array.size(); i++) {
        auto & name = name_array.at(i);
        auto itr = std::find_if(
          ros_msg.field_lines.begin(), ros_msg.field_lines.end(),
          std::bind(check_field_line_name, std::placeholders::_1, name));
        if (itr != ros_msg.field_lines.end()) {
          target_array.at(i).x = itr->p1.x;
          target_array.at(i).y = itr->p1.y;
          target_array.at(2 * i + 1).x = itr->p2.x;
          target_array.at(2 * i + 1).y = itr->p2.y;
        }
      }
    };

  std::array<std::string, 2> field_bound_names = {"TopTouchLine", "BottomTouchLine"};
  lines_to_points(field_bound_names, field_info.field_corners);

  ateam_msgs::msg::FieldSidedInfo left_side_info {};
  left_side_info.goal_posts.resize(2);
  left_side_info.goal_posts.at(0).x = -field_info.field_length / 2.0;
  left_side_info.goal_posts.at(0).y = field_info.goal_width / 2.0;
  left_side_info.goal_posts.at(1).x = -field_info.field_length / 2.0;
  left_side_info.goal_posts.at(1).y = -field_info.goal_width / 2.0;

  std::array<std::string,
    2> left_penalty_names = {"LeftFieldLeftPenaltyStretch", "LeftFieldRightPenaltyStretch"};
  lines_to_points(left_penalty_names, left_side_info.goalie_corners);


  ateam_msgs::msg::FieldSidedInfo right_side_info {};
  right_side_info.goal_posts.resize(2);
  right_side_info.goal_posts.at(0).x = field_info.field_length / 2.0;
  right_side_info.goal_posts.at(0).y = field_info.goal_width / 2.0;
  right_side_info.goal_posts.at(1).x = field_info.field_length / 2.0;
  right_side_info.goal_posts.at(1).y = -field_info.goal_width / 2.0;

  // note left and right can be different according to Joe
  // Temporary stupid assignment working under the assumption we are on left side
  // This gets inverted later if side is not right
  field_info.ours = left_side_info;
  field_info.theirs = right_side_info;
  return field_info;
}

// I hate all my code...
// because we really dont have access to the structs of the ros message just did it here
void invert_field_info(ateam_msgs::msg::FieldInfo & info)
{
  auto invert_point_array = [&](auto & target_array) {
      for (auto & point : target_array) {
        point.x *= -1.0;
        point.y *= -1.0;
      }
    };

  invert_point_array(info.field_corners);
  invert_point_array(info.ours.goalie_corners);
  invert_point_array(info.ours.goal_posts);
  invert_point_array(info.theirs.goalie_corners);
  invert_point_array(info.theirs.goal_posts);
}

CameraMeasurement fromMsg(const ssl_league_msgs::msg::VisionDetectionFrame & ros_msg)
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

}  // namespace ateam_vision_filter::message_conversions
