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

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <array>
#include <chrono>
#include <functional>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ateam_common/parameters.hpp>
#include <ateam_common/overlay.hpp>
#include <ateam_common/topic_names.hpp>
#include <ateam_common/game_controller_listener.hpp>
#include <ateam_common/indexed_topic_helpers.hpp>
#include <ateam_msgs/msg/ball_state.hpp>
#include <ateam_msgs/msg/robot_motion_command.hpp>
#include <ateam_msgs/msg/robot_state.hpp>
#include <ateam_msgs/msg/world.hpp>
#include <ateam_msgs/msg/field_info.hpp>
#include <ateam_msgs/msg/field_sided_info.hpp>
#include <ssl_league_msgs/msg/vision_geometry_field_size.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "behavior/behavior_evaluator.hpp"
#include "behavior/behavior_executor.hpp"
#include "behavior/behavior_follower.hpp"
#include "behavior/behavior_realization.hpp"
#include "trajectory_generation/trajectory_editor.hpp"
#include "types/world.hpp"
#include "util/directed_graph.hpp"
#include "util/message_conversions.hpp"

using namespace std::chrono_literals;
using ateam_common::indexed_topic_helpers::create_indexed_subscribers;
using ateam_common::indexed_topic_helpers::create_indexed_publishers;

namespace ateam_ai
{

class ATeamAINode : public rclcpp::Node
{
public:
  explicit ATeamAINode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("ateam_ai_node", options), game_controller_listener_(*this),
    evaluator_(realization_), executor_(realization_)
  {
    REGISTER_NODE_PARAMS(this);
    ateam_common::Overlay::GetOverlay().SetNamespace("ateam_ai");

    std::lock_guard<std::mutex> lock(world_mutex_);
    world_.balls.emplace_back(Ball{});

    create_indexed_subscribers<ateam_msgs::msg::RobotState>(
      blue_robots_subscriptions_,
      Topics::kBlueTeamRobotPrefix,
      10,
      &ATeamAINode::blue_robot_state_callback,
      this);

    create_indexed_subscribers<ateam_msgs::msg::RobotState>(
      yellow_robots_subscriptions_,
      Topics::kYellowTeamRobotPrefix,
      10,
      &ATeamAINode::yellow_robot_state_callback,
      this);

    create_indexed_publishers<ateam_msgs::msg::RobotMotionCommand>(
      robot_commands_publishers_, Topics::kRobotMotionCommandPrefix,
      rclcpp::SystemDefaultsQoS(), this);

    auto ball_callback = [&](const ateam_msgs::msg::BallState::SharedPtr ball_state_msg) {
        ball_state_callback(world_.balls.at(0), ball_state_msg);
      };
    ball_subscription_ = create_subscription<ateam_msgs::msg::BallState>(
      std::string(Topics::kBall),
      10,
      ball_callback);

    world_publisher_ = create_publisher<ateam_msgs::msg::World>(
      "~/world",
      rclcpp::SystemDefaultsQoS());

    overlay_publisher_ = create_publisher<ateam_msgs::msg::Overlay>(
      "/overlay",
      rclcpp::SystemDefaultsQoS());
    ateam_common::Overlay::GetOverlay().SetOverlayPublishCallback(
      [&](ateam_msgs::msg::Overlay overlay) {
        overlay_publisher_->publish(overlay);
      }
    );

    field_subscription_ = create_subscription<ateam_msgs::msg::FieldInfo>(
      std::string(Topics::kField),
      10,
      std::bind(&ATeamAINode::field_callback, this, std::placeholders::_1));

    timer_ = create_wall_timer(10ms, std::bind(&ATeamAINode::timer_callback, this));
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<ateam_msgs::msg::BallState>::SharedPtr ball_subscription_;
  std::array<rclcpp::Subscription<ateam_msgs::msg::RobotState>::SharedPtr,
    16> blue_robots_subscriptions_;
  std::array<rclcpp::Subscription<ateam_msgs::msg::RobotState>::SharedPtr,
    16> yellow_robots_subscriptions_;
  rclcpp::Subscription<ateam_msgs::msg::FieldInfo>::SharedPtr
    field_subscription_;
  std::array<rclcpp::Publisher<ateam_msgs::msg::RobotMotionCommand>::SharedPtr,
    16> robot_commands_publishers_;
  rclcpp::Publisher<ateam_msgs::msg::Overlay>::SharedPtr overlay_publisher_;

  rclcpp::Publisher<ateam_msgs::msg::World>::SharedPtr world_publisher_;

  ateam_common::GameControllerListener game_controller_listener_;

  BehaviorRealization realization_;
  BehaviorEvaluator evaluator_;
  BehaviorExecutor executor_;
  BehaviorFollower follower_;
  std::mutex world_mutex_;
  World world_;
  std::array<std::optional<Trajectory>, 16> previous_frame_trajectories;

  void blue_robot_state_callback(
    const ateam_msgs::msg::RobotState::SharedPtr robot_state_msg,
    int id)
  {
    const auto are_we_blue = game_controller_listener_.GetTeamColor() ==
      ateam_common::TeamColor::Blue;
    auto & robot_state_array = are_we_blue ? world_.our_robots : world_.their_robots;
    robot_state_callback(robot_state_array, id, robot_state_msg);
  }

  void yellow_robot_state_callback(
    const ateam_msgs::msg::RobotState::SharedPtr robot_state_msg,
    int id)
  {
    const auto are_we_yellow = game_controller_listener_.GetTeamColor() ==
      ateam_common::TeamColor::Yellow;
    auto & robot_state_array = are_we_yellow ? world_.our_robots : world_.their_robots;
    robot_state_callback(robot_state_array, id, robot_state_msg);
  }

  void robot_state_callback(
    std::array<std::optional<Robot>, 16> & robot_states,
    std::size_t id,
    const ateam_msgs::msg::RobotState::SharedPtr robot_state_msg)
  {
    std::lock_guard<std::mutex> lock(world_mutex_);
    robot_states.at(id) = Robot();
    robot_states.at(id).value().pos.x() = robot_state_msg->pose.position.x;
    robot_states.at(id).value().pos.y() = robot_state_msg->pose.position.y;
    tf2::Quaternion tf2_quat;
    tf2::fromMsg(robot_state_msg->pose.orientation, tf2_quat);
    robot_states.at(id).value().theta = tf2::getYaw(tf2_quat);
    robot_states.at(id).value().vel.x() = robot_state_msg->twist.linear.x;
    robot_states.at(id).value().vel.y() = robot_state_msg->twist.linear.y;
    robot_states.at(id).value().omega = robot_state_msg->twist.angular.z;
  }

  void ball_state_callback(
    Ball & ball_state,
    const ateam_msgs::msg::BallState::SharedPtr ball_state_msg)
  {
    std::lock_guard<std::mutex> lock(world_mutex_);
    ball_state.pos.x() = ball_state_msg->pose.position.x;
    ball_state.pos.y() = ball_state_msg->pose.position.y;
    ball_state.vel.x() = ball_state_msg->twist.linear.x;
    ball_state.vel.y() = ball_state_msg->twist.linear.y;
  }

  void field_callback(const ateam_msgs::msg::FieldInfo::SharedPtr field_msg)
  {
    Field field {
      .field_length = field_msg->field_length,
      .field_width = field_msg->field_width,
      .goal_width = field_msg->goal_width,
      .goal_depth = field_msg->goal_depth,
      .boundary_width = field_msg->boundary_width
    };

    // I could have just defined conversion operators for all of this but
    // Im pretty sure joe wanted ros separate from cpp
    auto convert_point_array = [&](auto & starting_array, auto final_array_iter) {
        std::transform(
          starting_array.begin(), starting_array.end(), final_array_iter,
          [&](auto & val)->Eigen::Vector2d {
            return {val.x, val.y};
          });
      };

    convert_point_array(field_msg->field_corners, field.field_corners.begin());
    convert_point_array(field_msg->ours.goal_posts, field.ours.goalie_corners.begin());
    convert_point_array(field_msg->ours.goalie_corners, field.ours.goal_posts.begin());
    convert_point_array(field_msg->theirs.goal_posts, field.theirs.goalie_corners.begin());
    convert_point_array(field_msg->theirs.goalie_corners, field.theirs.goal_posts.begin());

    std::lock_guard<std::mutex> lock(world_mutex_);
    world_.field = field;
  }

  void timer_callback()
  {
    std::lock_guard<std::mutex> lock(world_mutex_);

    //
    // Preproccess world
    //
    world_.current_time += 0.01;
    for (std::size_t robot_id = 0; robot_id < 16; robot_id++) {
      // Estimate of how long it will take for the round trip of
      // Command -> Radio -> Robot -> Motion -> Vision change
      const auto & maybe_trajectory = previous_frame_trajectories.at(robot_id);
      if (maybe_trajectory.has_value()) {
        world_.plan_from_our_robots.at(robot_id) = trajectory_editor::state_at_immutable_duration(
          maybe_trajectory.value(),
          world_.immutable_duration, world_.current_time);
      } else {
        world_.plan_from_our_robots.at(robot_id) = world_.our_robots.at(robot_id);
      }
    }

    // Get current game state for world
    world_.referee_info.running_command = game_controller_listener_.GetGameCommand();
    world_.referee_info.current_game_stage = game_controller_listener_.GetGameStage();
    world_.referee_info.our_goalie_id = game_controller_listener_.GetOurGoalieID().value_or(0); // current work has just made me into a self doubt pit I cant do anything right
    world_.referee_info.their_goalie_id = game_controller_listener_.GetTheirGoalieID().value_or(0);
    // Save off the world to the rosbag
    world_publisher_->publish(ateam_ai::message_conversions::toMsg(world_));

    //
    // Plan behavior
    //
    auto current_behaviors = evaluator_.get_best_behaviors(world_);
    auto current_trajectories = executor_.execute_behaviors(
      current_behaviors, world_,
      world_.behavior_executor_state);
    auto robot_motion_commands = follower_.follow(current_trajectories, world_);

    send_all_motion_commands(robot_motion_commands);

    //
    // Cleanup for next frame
    //
    previous_frame_trajectories = current_trajectories;
  }

  void send_all_motion_commands(
    const std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
    16> & robot_motion_commands)
  {
    for (std::size_t id = 0; id < robot_commands_publishers_.size(); id++) {
      const auto & maybe_motion_command = robot_motion_commands.at(id);
      if (maybe_motion_command.has_value()) {
        robot_commands_publishers_.at(id)->publish(maybe_motion_command.value());
      }
    }
  }
};

}  // namespace ateam_ai

RCLCPP_COMPONENTS_REGISTER_NODE(ateam_ai::ATeamAINode)
