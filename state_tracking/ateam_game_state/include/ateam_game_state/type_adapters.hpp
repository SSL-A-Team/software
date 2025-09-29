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


#ifndef ATEAM_GAME_STATE__TYPE_ADAPTERS_HPP_
#define ATEAM_GAME_STATE__TYPE_ADAPTERS_HPP_

#include <ateam_msgs/msg/world.hpp>
#include <ateam_msgs/msg/ball_state.hpp>
#include <ateam_msgs/msg/robot_state.hpp>
#include <ateam_msgs/msg/field_info.hpp>
#include <ateam_msgs/msg/referee_info.hpp>
#include <rclcpp/type_adapter.hpp>
#include "world.hpp"
#include "ball.hpp"
#include "robot.hpp"
#include "field.hpp"
#include "referee_info.hpp"


template<>
struct rclcpp::TypeAdapter<ateam_game_state::World, ateam_msgs::msg::World>
{
  using is_specialized = std::true_type;
  using custom_type = ateam_game_state::World;
  using ros_message_type = ateam_msgs::msg::World;

  static void convert_to_ros_message(const custom_type & world, ros_message_type & ros_msg);

  static void convert_to_custom(const ros_message_type & ros_msg, custom_type & world);
};

template<>
struct rclcpp::TypeAdapter<ateam_game_state::Ball, ateam_msgs::msg::BallState>
{
  using is_specialized = std::true_type;
  using custom_type = ateam_game_state::Ball;
  using ros_message_type = ateam_msgs::msg::BallState;

  static void convert_to_ros_message(const custom_type & ball, ros_message_type & ros_msg);

  static void convert_to_custom(const ros_message_type & ros_msg, custom_type & ball);
};

template<>
struct rclcpp::TypeAdapter<ateam_game_state::Robot, ateam_msgs::msg::RobotState>
{
  using is_specialized = std::true_type;
  using custom_type = ateam_game_state::Robot;
  using ros_message_type = ateam_msgs::msg::RobotState;

  static void convert_to_ros_message(const custom_type & robot, ros_message_type & ros_msg);

  static void convert_to_custom(const ros_message_type & ros_msg, custom_type & robot);
};

template<>
struct rclcpp::TypeAdapter<ateam_game_state::Field, ateam_msgs::msg::FieldInfo>
{
  using is_specialized = std::true_type;
  using custom_type = ateam_game_state::Field;
  using ros_message_type = ateam_msgs::msg::FieldInfo;

  static void convert_to_ros_message(const custom_type & field, ros_message_type & ros_msg);

  static void convert_to_custom(const ros_message_type & ros_msg, custom_type & field);
};

template<>
struct rclcpp::TypeAdapter<ateam_game_state::RefereeInfo, ateam_msgs::msg::RefereeInfo>
{
  using is_specialized = std::true_type;
  using custom_type = ateam_game_state::RefereeInfo;
  using ros_message_type = ateam_msgs::msg::RefereeInfo;

  static void convert_to_ros_message(const custom_type & ref_info, ros_message_type & ros_msg);

  static void convert_to_custom(const ros_message_type & ros_msg, custom_type & ref_info);
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(ateam_game_state::World, ateam_msgs::msg::World);
RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(ateam_game_state::Ball, ateam_msgs::msg::BallState);
RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(ateam_game_state::Robot, ateam_msgs::msg::RobotState);
RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(ateam_game_state::Field, ateam_msgs::msg::FieldInfo);
RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(ateam_game_state::RefereeInfo, ateam_msgs::msg::RefereeInfo);

#endif  // ATEAM_GAME_STATE__TYPE_ADAPTERS_HPP_
