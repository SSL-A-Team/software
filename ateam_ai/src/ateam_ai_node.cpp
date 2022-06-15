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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ateam_msgs/msg/ball_state.hpp>
#include <ateam_msgs/msg/robot_motion_command.hpp>
#include <ateam_msgs/msg/robot_state.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <functional>
#include <mutex>


#include "behavior/behavior.hpp"
#include "behavior/behavior_feedback.hpp"
#include "behavior/behavior_evaluator.hpp"
#include "behavior/behavior_executor.hpp"
#include "behavior/behavior_realization.hpp"
#include "types/world.hpp"
#include "util/directed_graph.hpp"

using namespace std::chrono_literals;

namespace ateam_ai
{

class ATeamAINode : public rclcpp::Node
{
public:
  explicit ATeamAINode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("ateam_ai_node", options), evaluator_(realization_), executor_(realization_)
  {
    std::lock_guard<std::mutex> lock(world_mutex_);
    world_.balls.emplace_back(Ball{});

    for (std::size_t id = 0; id < blue_robots_subscriptions_.size(); id++) {
      auto our_robot_callback =
        [&, id](const ateam_msgs::msg::RobotState::SharedPtr robot_state_msg) {
          robot_state_callback(world_.our_robots, id, robot_state_msg);
        };
      auto their_robot_callback =
        [&, id](const ateam_msgs::msg::RobotState::SharedPtr robot_state_msg) {
          robot_state_callback(world_.their_robots, id, robot_state_msg);
        };
      blue_robots_subscriptions_.at(id) = create_subscription<ateam_msgs::msg::RobotState>(
        "/vision_filter/blue_team/robot" + std::to_string(id),
        10,
        our_robot_callback);
      yellow_robots_subscriptions_.at(id) = create_subscription<ateam_msgs::msg::RobotState>(
        "/vision_filter/yellow_team/robot" + std::to_string(id),
        10,
        their_robot_callback);
    }

    for (std::size_t id = 0; id < robot_commands_publishers_.size(); id++) {
      robot_commands_publishers_.at(id) = create_publisher<ateam_msgs::msg::RobotMotionCommand>(
        "~/robot_motion_commands/robot" + std::to_string(id),
        rclcpp::SystemDefaultsQoS());
    }

    auto ball_callback = [&](const ateam_msgs::msg::BallState::SharedPtr ball_state_msg) {
        ball_state_callback(world_.balls.at(0), ball_state_msg);
      };
    ball_subscription_ = create_subscription<ateam_msgs::msg::BallState>(
      "/vision_filter/ball",
      10,
      ball_callback);

    timer_ = create_wall_timer(10ms, std::bind(&ATeamAINode::timer_callback, this));
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<ateam_msgs::msg::BallState>::SharedPtr ball_subscription_;
  std::array<rclcpp::Subscription<ateam_msgs::msg::RobotState>::SharedPtr,
    16> blue_robots_subscriptions_;
  std::array<rclcpp::Subscription<ateam_msgs::msg::RobotState>::SharedPtr,
    16> yellow_robots_subscriptions_;
  std::array<rclcpp::Publisher<ateam_msgs::msg::RobotMotionCommand>::SharedPtr,
    16> robot_commands_publishers_;

  BehaviorRealization realization_;
  BehaviorEvaluator evaluator_;
  BehaviorExecutor executor_;
  std::mutex world_mutex_;
  World world_;

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
    robot_states.at(id).value().theta = tf2_quat.getAngle();
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

  void timer_callback()
  {
    std::lock_guard<std::mutex> lock(world_mutex_);
    DirectedGraph<Behavior> current_behaviors;

    current_behaviors = evaluator_.get_best_behaviors(world_);
    auto robot_motion_commands = executor_.execute_behaviors(current_behaviors, world_);

    send_all_motion_commands(robot_motion_commands);
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
