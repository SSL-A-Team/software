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

#include "behavior/behavior_executor.hpp"

#include "behavior/behavior_feedback.hpp"

BehaviorExecutor::BehaviorExecutor(
  BehaviorRealization & behavior_realization,
  rclcpp::Publisher<ateam_msgs::msg::RobotMotionCommand>::SharedPtr robot_commands)
: behavior_realization(behavior_realization), robot_commands(robot_commands) {}

void BehaviorExecutor::execute_behaviors(
  const DirectedGraph<Behavior> & behaviors,
  const World & world)
{
  //
  // Grab trajectories for everything
  //
  DirectedGraph<BehaviorFeedback> behavior_feedback = behavior_realization.realize_behaviors(
    behaviors, world);

  //
  // Replan course trajectories as we approach their start time
  //

  // Long term planning (>10 seconds) is not valid due to the speed of robots
  // as we approach some of the later behaviors in time, we need to replan them
  // using better planners to dodge these obsticles and everything

  //
  // Follow trajectories
  //

  // Send commands down to motion control
  Eigen::Vector2d target = std::get<MoveParam>(
    behaviors.get_node(
      behaviors.get_root_nodes().front()).params).target_location;
  Eigen::Vector2d command = (target - world.our_robots.at(0).value_or(Robot()).pos) / 1000.0;
  if (command.norm() > 1) {
    command = command.normalized();
  }
  ateam_msgs::msg::RobotMotionCommand motion_command;
  motion_command.twist.linear.x = command.x();
  motion_command.twist.linear.y = command.y();
  robot_commands->publish(motion_command);
}
