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

#ifndef BEHAVIOR__BEHAVIOR_EXECUTOR_HPP_
#define BEHAVIOR__BEHAVIOR_EXECUTOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <ateam_msgs/msg/robot_motion_command.hpp>

#include "behavior/behavior.hpp"
#include "behavior/behavior_realization.hpp"
#include "types/world.hpp"
#include "util/directed_graph.hpp"

/**
 * Given a set of behaviors
 *  - Replan trajectories as their start time approaches
 *  - Manage/Follow the trajectories
 */
class BehaviorExecutor
{
public:
  explicit BehaviorExecutor(
    BehaviorRealization & behavior_realization,
    rclcpp::Publisher<ateam_msgs::msg::RobotMotionCommand>::SharedPtr robot_commands);

  void execute_behaviors(const DirectedGraph<Behavior> & behaviors, const World & world);

private:
  BehaviorRealization & behavior_realization;
  rclcpp::Publisher<ateam_msgs::msg::RobotMotionCommand>::SharedPtr robot_commands;
};

#endif  // BEHAVIOR__BEHAVIOR_EXECUTOR_HPP_
