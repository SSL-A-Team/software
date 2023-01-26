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

#ifndef BEHAVIOR__BEHAVIOR_FOLLOWER_HPP_
#define BEHAVIOR__BEHAVIOR_FOLLOWER_HPP_

#include <optional>
#include <array>

#include <rclcpp/rclcpp.hpp>
#include <ateam_msgs/msg/robot_motion_command.hpp>

#include "behavior/behavior_realization.hpp"
#include "types/trajectory.hpp"
#include "types/world.hpp"

/**
 * Given trajectories as a function of time
 *  - Follow them as best as possible
 */
class BehaviorFollower
{
public:
  using MaybeRobotMotionCommand = std::optional<ateam_msgs::msg::RobotMotionCommand>;
  using RobotMotionCommands = std::array<MaybeRobotMotionCommand, 16>;

  RobotMotionCommands follow(
    const std::array<std::optional<Trajectory>, 16> & robot_trajectories,
    World & world);

private:
  static Sample3d get_next_command(const Trajectory & t, double current_time);
};

#endif  // BEHAVIOR__BEHAVIOR_FOLLOWER_HPP_
