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


#include <ateam_common/game_controller_listener.hpp>
#include <ateam_common/indexed_topic_helpers.hpp>
#include <ateam_common/topic_names.hpp>
#include <ateam_msgs/msg/vision_state_ball.hpp>
#include <ateam_msgs/msg/vision_state_robot.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/utils.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "ateam_game_state/world.hpp"
#include "ateam_game_state/type_adapters.hpp"
#include "double_touch_evaluator.hpp"
#include "in_play_evaluator.hpp"


using ateam_common::indexed_topic_helpers::create_indexed_publishers;
using ateam_common::indexed_topic_helpers::create_indexed_subscribers;

namespace ateam_game_state
{

class GameStateTracker : public rclcpp::Node {
public:
  explicit GameStateTracker(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("game_state_tracker", options),
    gc_listener_(*this)
  {
    world_pub_ =
      create_publisher<World>(std::string(Topics::kWorld),
        rclcpp::QoS(1).best_effort().durability_volatile());

    field_sub_ = create_subscription<Field>(
      std::string(Topics::kField),
      1,
      std::bind(&GameStateTracker::FieldCallback, this, std::placeholders::_1));

    ball_sub_ = create_subscription<ateam_msgs::msg::VisionStateBall>(
      std::string(Topics::kBall),
      1,
      std::bind(&GameStateTracker::BallCallback, this, std::placeholders::_1));

    create_indexed_subscribers<ateam_msgs::msg::VisionStateRobot>(
      blue_bot_subs_,
      Topics::kBlueTeamRobotPrefix,
      1,
      &GameStateTracker::BlueVisionBotCallback,
      this);

    create_indexed_subscribers<ateam_msgs::msg::VisionStateRobot>(
      yellow_bot_subs_,
      Topics::kYellowTeamRobotPrefix,
      1,
      &GameStateTracker::YellowVisionBotCallback,
      this);

    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 /
        declare_parameter<double>("update_frequency")),
        std::bind(&GameStateTracker::TimerCallback, this));
  }

private:
  DoubleTouchEvaluator double_touch_evaluator_;
  InPlayEvaluator in_play_evaluator_;
  rclcpp::Publisher<World>::SharedPtr world_pub_;
  rclcpp::Subscription<Field>::SharedPtr field_sub_;
  rclcpp::Subscription<ateam_msgs::msg::VisionStateBall>::SharedPtr ball_sub_;
  std::array<rclcpp::Subscription<ateam_msgs::msg::VisionStateRobot>::SharedPtr,
    ateam_common::indexed_topic_helpers::kRobotCount> blue_bot_subs_;
  std::array<rclcpp::Subscription<ateam_msgs::msg::VisionStateRobot>::SharedPtr,
    ateam_common::indexed_topic_helpers::kRobotCount> yellow_bot_subs_;
  ateam_common::GameControllerListener gc_listener_;  // TODO(barulicm): Move GCListener type to ateam_game_state package
  World world_;
  rclcpp::TimerBase::SharedPtr timer_;

  void InitializeRobotIds()
  {
    for(auto i = 0u; i < world_.our_robots.size(); ++i) {
      world_.our_robots[i].id = i;
    }
    for(auto i = 0u; i < world_.their_robots.size(); ++i) {
      world_.their_robots[i].id = i;
    }
  }

  void FieldCallback(const std::unique_ptr<Field> & field)
  {
    world_.field = *field;
  }

  void BallCallback(const std::unique_ptr<ateam_msgs::msg::VisionStateBall> & msg)
  {
    world_.ball.pos = ateam_geometry::Point(msg->pose.position.x, msg->pose.position.y);
    world_.ball.vel = ateam_geometry::Vector(msg->twist.linear.x, msg->twist.linear.y);
    world_.ball.visible = msg->visible;
    // TODO(barulicm): Update remaining Ball fields
  }

  void UpdateBotFromVision(Robot & robot, const ateam_msgs::msg::VisionStateRobot::SharedPtr msg)
  {
    robot.visible = msg->visible;
    if (msg->visible) {
      robot.pos = ateam_geometry::Point(
        msg->pose.position.x,
        msg->pose.position.y);
      tf2::Quaternion tf2_quat;
      tf2::fromMsg(msg->pose.orientation, tf2_quat);
      robot.theta = tf2::getYaw(tf2_quat);
      robot.vel = ateam_geometry::Vector(
        msg->twist.linear.x,
        msg->twist.linear.y);
      robot.omega = msg->twist.angular.z;
    }
  }

  void BlueVisionBotCallback(const ateam_msgs::msg::VisionStateRobot::SharedPtr msg, int id)
  {
    const auto our_color = gc_listener_.GetTeamColor();
    if(our_color == ateam_common::TeamColor::Unknown) {
      return;
    }
    const auto are_we_blue = our_color == ateam_common::TeamColor::Blue;
    auto & robot_state_array = are_we_blue ? world_.our_robots : world_.their_robots;
    UpdateBotFromVision(robot_state_array[id], msg);
  }

  void YellowVisionBotCallback(const ateam_msgs::msg::VisionStateRobot::SharedPtr msg, int id)
  {
    const auto our_color = gc_listener_.GetTeamColor();
    if(our_color == ateam_common::TeamColor::Unknown) {
      return;
    }
    const auto are_we_yellow = our_color == ateam_common::TeamColor::Yellow;
    auto & robot_state_array = are_we_yellow ? world_.our_robots : world_.their_robots;
    UpdateBotFromVision(robot_state_array[id], msg);
  }

  void UpdateRefInfo()
  {
    world_.referee_info.our_goalie_id = gc_listener_.GetOurGoalieID().value_or(-1);
    world_.referee_info.their_goalie_id = gc_listener_.GetTheirGoalieID().value_or(-1);
    world_.referee_info.game_stage = gc_listener_.GetGameStage();
    world_.referee_info.running_command = gc_listener_.GetGameCommand();
    world_.referee_info.prev_command = gc_listener_.GetPreviousGameCommand();

    const auto & gc_designated_position =
      gc_listener_.GetDesignatedPosition();
    if (gc_designated_position.has_value()) {
      if (gc_listener_.GetTeamSide() == ateam_common::TeamSide::PositiveHalf) {
        world_.referee_info.designated_position = ateam_geometry::Point(
        -gc_designated_position->x,
        -gc_designated_position->y);
      } else {
        world_.referee_info.designated_position = ateam_geometry::Point(
        gc_designated_position->x,
        gc_designated_position->y);
      }
    } else {
      world_.referee_info.designated_position = std::nullopt;
    }

    world_.referee_info.next_command = gc_listener_.GetNextGameCommand();

    const auto & ref_msg = gc_listener_.GetLatestRefereeMessage();
    world_.referee_info.command_time =
      std::chrono::system_clock::time_point(
      std::chrono::nanoseconds(
        rclcpp::Time(ref_msg.command_timestamp).nanoseconds()));
  }

  void TimerCallback()
  {
    UpdateRefInfo();
    double_touch_evaluator_.Update(world_);
    in_play_evaluator_.Update(world_);
    world_pub_->publish(world_);
  }
};

}  // namespace ateam_game_state

RCLCPP_COMPONENTS_REGISTER_NODE(ateam_game_state::GameStateTracker)
