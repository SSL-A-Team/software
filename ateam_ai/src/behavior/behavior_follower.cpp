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

#include "behavior/behavior_follower.hpp"

BehaviorFollower::RobotMotionCommands BehaviorFollower::follow(
  const std::array<std::optional<Trajectory>, 16> & robot_trajectories,
  World & world)
{
  RobotMotionCommands robot_motion_commands;
  for (std::size_t robot_id = 0; robot_id < 16; robot_id++) {
    const auto & maybe_trajectory = robot_trajectories.at(robot_id);

    // We need a trajectory that is at least 1 sample long
    if (!maybe_trajectory.has_value() ||
      maybe_trajectory.value().samples.empty())
    {
      continue;
    }

    Sample3d command = get_next_command(maybe_trajectory.value(), world.current_time);

    ateam_msgs::msg::RobotMotionCommand motion_command;
    double kp = 0.5;
    motion_command.twist.linear.x = command.vel.x() +
      kp*(command.pose.x() - world.our_robots.at(robot_id).value().pos.x());
    motion_command.twist.linear.y = command.vel.y() +
      kp*(command.pose.y() - world.our_robots.at(robot_id).value().pos.y());
    double theta_diff = command.pose.z() - world.our_robots.at(robot_id).value().theta;
    motion_command.twist.angular.z = command.vel.z() +
      kp*(atan2(sin(theta_diff), cos(theta_diff)));

    std::cout << command.pose.z() << " " << world.our_robots.at(robot_id).value().theta << std::endl;
    Eigen::Vector2d robot{world.our_robots.at(robot_id).value().pos.x(), world.our_robots.at(robot_id).value().pos.y()};
    motion_command.kick = world.get_unique_ball().has_value() && (world.get_unique_ball().value().pos - robot).norm() < 0.1;
    robot_motion_commands.at(robot_id) = motion_command;
  }

  return robot_motion_commands;
}

Sample3d BehaviorFollower::get_next_command(const Trajectory & t, double current_time)
{
  Sample3d command;
  command = t.samples.front();

  // Find first sample that is either current time or after
  for (const auto & sample : t.samples) {
    if (sample.time >= current_time) {
      command = sample;
      break;
    }
  }

  return command;
}
