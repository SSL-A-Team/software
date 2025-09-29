// Copyright 2025 A Team
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


#include "ateam_game_state/type_adapters.hpp"

#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

void rclcpp::TypeAdapter<ateam_game_state::World, ateam_msgs::msg::World>::convert_to_ros_message(
  const custom_type & world, ros_message_type & ros_msg)
{
  using FieldTA = rclcpp::adapt_type<ateam_game_state::Field>::as<ateam_msgs::msg::FieldInfo>;
  using RobotTA = rclcpp::adapt_type<ateam_game_state::Robot>::as<ateam_msgs::msg::RobotState>;
  using BallTA = rclcpp::adapt_type<ateam_game_state::Ball>::as<ateam_msgs::msg::BallState>;
  using RefTA = rclcpp::adapt_type<ateam_game_state::RefereeInfo>::as<ateam_msgs::msg::RefereeInfo>;

  ros_msg.current_time =
    rclcpp::Time(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      world.current_time.time_since_epoch()).count());

  FieldTA::convert_to_ros_message(world.field, ros_msg.field);

  RefTA::convert_to_ros_message(world.referee_info, ros_msg.referee_info);

  BallTA::convert_to_ros_message(world.ball, ros_msg.balls.emplace_back());

  ros_msg.our_robots.reserve(world.our_robots.size());
  for (const ateam_game_state::Robot & robot : world.our_robots) {
    if (robot.visible || robot.radio_connected) {
      RobotTA::convert_to_ros_message(robot, ros_msg.our_robots.emplace_back());
    } else {
      ros_msg.our_robots.push_back(ateam_msgs::msg::RobotState());
    }
  }

  ros_msg.their_robots.reserve(world.their_robots.size());
  for (const ateam_game_state::Robot & robot : world.their_robots) {
    if (robot.visible) {
      RobotTA::convert_to_ros_message(robot, ros_msg.their_robots.emplace_back());
    } else {
      ros_msg.their_robots.push_back(ateam_msgs::msg::RobotState());
    }
  }

  ros_msg.ball_in_play = world.in_play;
  ros_msg.double_touch_enforced = world.double_touch_forbidden_id_.has_value();
  ros_msg.double_touch_id = world.double_touch_forbidden_id_.value_or(-1);
}

void rclcpp::TypeAdapter<ateam_game_state::World, ateam_msgs::msg::World>::convert_to_custom(
  const ros_message_type & ros_msg, custom_type & world)
{
  using FieldTA = rclcpp::adapt_type<ateam_game_state::Field>::as<ateam_msgs::msg::FieldInfo>;
  using RobotTA = rclcpp::adapt_type<ateam_game_state::Robot>::as<ateam_msgs::msg::RobotState>;
  using BallTA = rclcpp::adapt_type<ateam_game_state::Ball>::as<ateam_msgs::msg::BallState>;
  using RefTA = rclcpp::adapt_type<ateam_game_state::RefereeInfo>::as<ateam_msgs::msg::RefereeInfo>;

  world.current_time =
    std::chrono::steady_clock::time_point(std::chrono::seconds(ros_msg.current_time.sec) +
    std::chrono::nanoseconds(ros_msg.current_time.nanosec));

  FieldTA::convert_to_custom(ros_msg.field, world.field);

  RefTA::convert_to_custom(ros_msg.referee_info, world.referee_info);

  BallTA::convert_to_custom(ros_msg.balls.front(), world.ball);

  const auto convert_bot = [](const auto & ros_robot) {
      ateam_game_state::Robot robot;
      RobotTA::convert_to_custom(ros_robot, robot);
      return robot;
    };

  const auto our_robots_count = std::min(ros_msg.our_robots.size(), world.our_robots.size());
  std::transform(ros_msg.our_robots.begin(), ros_msg.our_robots.begin() + our_robots_count,
    world.our_robots.begin(), convert_bot);

  const auto their_robots_count = std::min(ros_msg.their_robots.size(), world.their_robots.size());
  std::transform(ros_msg.their_robots.begin(), ros_msg.their_robots.begin() + their_robots_count,
    world.their_robots.begin(), convert_bot);

  world.in_play = ros_msg.ball_in_play;
  if(ros_msg.double_touch_enforced) {
    world.double_touch_forbidden_id_ = ros_msg.double_touch_id;
  }
}

void rclcpp::TypeAdapter<ateam_game_state::Ball,
  ateam_msgs::msg::BallState>::convert_to_ros_message(
  const custom_type & ball, ros_message_type & ros_msg)
{
  ros_msg.pose.position.x = ball.pos.x();
  ros_msg.pose.position.y = ball.pos.y();
  ros_msg.twist.linear.x = ball.vel.x();
  ros_msg.twist.linear.y = ball.vel.y();
  ros_msg.visible = ball.visible;
}

void rclcpp::TypeAdapter<ateam_game_state::Ball, ateam_msgs::msg::BallState>::convert_to_custom(
  const ros_message_type & ros_msg, custom_type & ball)
{
  ball.pos = ateam_geometry::Point(ros_msg.pose.position.x, ros_msg.pose.position.y);
  ball.vel = ateam_geometry::Vector(ros_msg.twist.linear.x, ros_msg.twist.linear.y);
  ball.visible = ros_msg.visible;
}

void rclcpp::TypeAdapter<ateam_game_state::Robot,
  ateam_msgs::msg::RobotState>::convert_to_ros_message(
  const custom_type & robot, ros_message_type & ros_msg)
{
  ros_msg.pose.position.x = robot.pos.x();
  ros_msg.pose.position.y = robot.pos.y();
  ros_msg.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), robot.theta));
  ros_msg.twist.linear.x = robot.vel.x();
  ros_msg.twist.linear.y = robot.vel.y();
  ros_msg.twist.angular.z = robot.omega;
  ros_msg.visible = robot.visible;
}

void rclcpp::TypeAdapter<ateam_game_state::Robot, ateam_msgs::msg::RobotState>::convert_to_custom(
  const ros_message_type & ros_msg, custom_type & robot)
{
  robot.pos = ateam_geometry::Point(ros_msg.pose.position.x, ros_msg.pose.position.y);
  tf2::Quaternion quat;
  tf2::fromMsg(ros_msg.pose.orientation, quat);
  robot.theta = tf2::getYaw(quat);
  robot.vel = ateam_geometry::Vector(ros_msg.twist.linear.x, ros_msg.twist.linear.y);
  robot.omega = ros_msg.twist.angular.z;
  robot.visible = ros_msg.visible;
}

void rclcpp::TypeAdapter<ateam_game_state::Field,
  ateam_msgs::msg::FieldInfo>::convert_to_ros_message(
  const custom_type & field, ros_message_type & ros_msg)
{
  ros_msg.field_length = field.field_length;
  ros_msg.field_width = field.field_width;
  ros_msg.goal_width = field.goal_width;
  ros_msg.goal_depth = field.goal_depth;
  ros_msg.boundary_width = field.boundary_width;
  ros_msg.ignore_side = field.ignore_side;

  auto convert_point_array = [&](auto & in_array, auto out_array_iter) {
      std::transform(
        in_array.begin(), in_array.end(), out_array_iter,
        [&](auto & val)->geometry_msgs::msg::Point32 {
          return geometry_msgs::build<geometry_msgs::msg::Point32>().x(val.x()).y(val.y()).z(0);
        });
    };

  convert_point_array(field.field_corners, std::back_inserter(ros_msg.field_corners.points));
  convert_point_array(
    field.ours.defense_area_corners,
    std::back_inserter(ros_msg.ours.defense_area_corners.points));
  convert_point_array(
    field.ours.goal_corners,
    std::back_inserter(ros_msg.ours.goal_corners.points));
  convert_point_array(
    field.theirs.defense_area_corners,
    std::back_inserter(ros_msg.theirs.defense_area_corners.points));
  convert_point_array(
    field.theirs.goal_corners,
    std::back_inserter(ros_msg.theirs.goal_corners.points));
}

void rclcpp::TypeAdapter<ateam_game_state::Field, ateam_msgs::msg::FieldInfo>::convert_to_custom(
  const ros_message_type & ros_msg, custom_type & field)
{
  field.field_length = ros_msg.field_length;
  field.field_width = ros_msg.field_width;
  field.goal_width = ros_msg.goal_width;
  field.goal_depth = ros_msg.goal_depth;
  field.boundary_width = ros_msg.boundary_width;
  field.ignore_side = ros_msg.ignore_side;

  auto convert_point_array = [&](auto & in_vector, auto out_array) {
      const auto count = std::min(in_vector.size(), out_array.size());
      std::transform(
      in_vector.begin(), in_vector.begin() + count, out_array.begin(),
        [&](auto & val)->ateam_geometry::Point {
          return ateam_geometry::Point(val.x, val.y);
      });
    };

  convert_point_array(ros_msg.field_corners.points, field.field_corners);
  convert_point_array(ros_msg.ours.defense_area_corners.points, field.ours.defense_area_corners);
  convert_point_array(ros_msg.ours.goal_corners.points, field.ours.goal_corners);
  convert_point_array(ros_msg.theirs.defense_area_corners.points,
    field.theirs.defense_area_corners);
  convert_point_array(ros_msg.theirs.goal_corners.points, field.theirs.goal_corners);
}

void rclcpp::TypeAdapter<ateam_game_state::RefereeInfo,
  ateam_msgs::msg::RefereeInfo>::convert_to_ros_message(const custom_type & ref_info,
  ros_message_type & ros_msg)
{
  ros_msg.our_goalie_id = ref_info.our_goalie_id;
  ros_msg.their_goalie_id = ref_info.their_goalie_id;
  ros_msg.game_stage = static_cast<uint8_t>(ref_info.current_game_stage);
  ros_msg.game_command = static_cast<uint8_t>(ref_info.running_command);
  ros_msg.prev_command = static_cast<uint8_t>(ref_info.prev_command);

  if (ref_info.designated_position.has_value()) {
    ros_msg.designated_position.x = ref_info.designated_position->x();
    ros_msg.designated_position.y = ref_info.designated_position->y();
  } else {
    ros_msg.designated_position.x = 0.0f;
    ros_msg.designated_position.y = 0.0f;
  }
}

void rclcpp::TypeAdapter<ateam_game_state::RefereeInfo,
  ateam_msgs::msg::RefereeInfo>::convert_to_custom(const ros_message_type & ros_msg,
  custom_type & ref_info)
{
  ref_info.our_goalie_id = ros_msg.our_goalie_id;
  ref_info.their_goalie_id = ros_msg.their_goalie_id;

  ref_info.current_game_stage = static_cast<ateam_common::GameStage>(ros_msg.game_stage);
  ref_info.running_command = static_cast<ateam_common::GameCommand>(ros_msg.game_command);
  ref_info.prev_command = static_cast<ateam_common::GameCommand>(ros_msg.prev_command);

  // TODO(barulicm): add 'has_designated_position' field
  ref_info.designated_position = ateam_geometry::Point(ros_msg.designated_position.x,
    ros_msg.designated_position.y);
}
