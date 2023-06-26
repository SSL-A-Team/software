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

#include <chrono>
#include <functional>
#include <mutex>
#include <iostream>
#include <string>

#include <ateam_common/topic_names.hpp>
#include <ateam_common/indexed_topic_helpers.hpp>
#include <ateam_common/game_controller_listener.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "world.hpp"
#include "message_conversions.hpp"

using namespace std::chrono_literals;

namespace ateam_vision_filter
{

class VisionFilterNode : public rclcpp::Node
{
public:
  explicit VisionFilterNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("ateam_vision_filter", options),
    info_listener_(*this)
  {
    timer_ = create_wall_timer(10ms, std::bind(&VisionFilterNode::timer_callback, this));

    ball_publisher_ = create_publisher<ateam_msgs::msg::BallState>(
      std::string(Topics::kBall),
      rclcpp::SystemDefaultsQoS());

    ateam_common::indexed_topic_helpers::create_indexed_publishers
    <ateam_msgs::msg::RobotState>(
      blue_robots_publisher_,
      Topics::kBlueTeamRobotPrefix,
      rclcpp::SystemDefaultsQoS(),
      this
    );
    ateam_common::indexed_topic_helpers::create_indexed_publishers
    <ateam_msgs::msg::RobotState>(
      yellow_robots_publisher_,
      Topics::kYellowTeamRobotPrefix,
      rclcpp::SystemDefaultsQoS(),
      this
    );

    vision_state_publisher_ = create_publisher<ateam_msgs::msg::VisionWorldState>(
      std::string(Topics::kVisionState),
      rclcpp::SystemDefaultsQoS());

    field_publisher_ = create_publisher<ateam_msgs::msg::FieldInfo>(
      std::string(Topics::kField),
      rclcpp::SystemDefaultsQoS());

    ssl_vision_subscription_ =
      create_subscription<ssl_league_msgs::msg::VisionWrapper>(
      std::string(Topics::kVisionMessages),
      10,
      std::bind(&VisionFilterNode::vision_callback, this, std::placeholders::_1));
  }

  void vision_callback(
    const ssl_league_msgs::msg::VisionWrapper::SharedPtr vision_wrapper_msg)
  {
    int camera_id = vision_wrapper_msg->detection.camera_id;
    CameraMeasurement camera_measurement = message_conversions::getCameraMeasurement(
      *vision_wrapper_msg);
    ateam_msgs::msg::FieldInfo field_msg =
      message_conversions::getFieldGeometry(*vision_wrapper_msg);

    // Our field convention is we should always been on the negative half.
    // So if this is positive for our team we should invert coords
    if (info_listener_.GetTeamSide() == ateam_common::TeamSide::PositiveHalf) {
      camera_measurement.invert();
      message_conversions::invert_field_info(field_msg);
    }

    {
      const std::lock_guard<std::mutex> lock(world_mutex_);
      world_.update_camera(camera_id, camera_measurement);
    }

    if (field_msg.valid) {
      field_publisher_->publish(field_msg);
    }
  }

  void timer_callback()
  {
    const std::lock_guard<std::mutex> lock(world_mutex_);
    vision_state_publisher_->publish(world_.get_vision_world_state());

    world_.predict();

    std::optional<Ball> maybe_ball = world_.get_ball_estimate();
    if (maybe_ball.has_value()) {
      ball_publisher_->publish(message_conversions::toMsg(maybe_ball.value()));
    }

    std::array<std::optional<Robot>, 16> blue_team_robots = world_.get_blue_robots_estimate();
    for (std::size_t id = 0; id < 16; id++) {
      const auto & maybe_robot = blue_team_robots.at(id);
      if (maybe_robot.has_value()) {
        blue_robots_publisher_.at(id)->publish(message_conversions::toMsg(maybe_robot.value()));
      }
    }

    std::array<std::optional<Robot>, 16> yellow_team_robots = world_.get_yellow_robots_estimate();
    for (std::size_t id = 0; id < 16; id++) {
      const auto & maybe_robot = yellow_team_robots.at(id);
      if (maybe_robot.has_value()) {
        yellow_robots_publisher_.at(id)->publish(message_conversions::toMsg(maybe_robot.value()));
      }
    }
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<ateam_msgs::msg::BallState>::SharedPtr ball_publisher_;
  std::array<rclcpp::Publisher<ateam_msgs::msg::RobotState>::SharedPtr, 16> blue_robots_publisher_;
  std::array<rclcpp::Publisher<ateam_msgs::msg::RobotState>::SharedPtr,
    16> yellow_robots_publisher_;
  rclcpp::Publisher<ateam_msgs::msg::VisionWorldState>::SharedPtr vision_state_publisher_;
  rclcpp::Publisher<ateam_msgs::msg::FieldInfo>::SharedPtr field_publisher_;
  rclcpp::Subscription<ssl_league_msgs::msg::VisionWrapper>::SharedPtr ssl_vision_subscription_;
  rclcpp::Subscription<ssl_league_msgs::msg::VisionWrapper>::SharedPtr ssl_vision_subs_;
  ateam_common::GameControllerListener info_listener_;

  std::mutex world_mutex_;
  World world_;

  // std::mutex field_mutex_;
  // ateam_msgs::msg::FieldInfo field_msg_;
};
}  // namespace ateam_vision_filter

RCLCPP_COMPONENTS_REGISTER_NODE(ateam_vision_filter::VisionFilterNode)
