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

#include <ateam_common/angle.hpp>
#include <ateam_common/parameters.hpp>

#include "util/pid.hpp"
#include "util/viz.hpp"

CREATE_PARAM(double, "follower/pid/x_kp", x_kp, 2);
CREATE_PARAM(double, "follower/pid/y_kp", y_kp, 2);
CREATE_PARAM(double, "follower/pid/t_kp", t_kp, 3);

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

    auto & robot_controllers = trajectory_controllers.at(robot_id);
    auto & x_controller = robot_controllers.at(0);
    x_controller.set_kp(x_kp);
    auto & y_controller = robot_controllers.at(1);
    y_controller.set_kp(y_kp);
    auto & t_controller = robot_controllers.at(2);
    t_controller.set_kp(t_kp);

    Sample3d command = get_next_command(maybe_trajectory.value(), world.current_time);

    const auto & our_robot = world.our_robots.at(robot_id).value();

    ateam_msgs::msg::RobotMotionCommand motion_command;
    motion_command.twist.linear.x = command.vel.x() + x_controller.execute(
      command.pose.x(), our_robot.pos.x());
    motion_command.twist.linear.y = command.vel.y() + y_controller.execute(
      command.pose.y(), our_robot.pos.y());
    motion_command.twist.angular.z = command.vel.z() + t_controller.execute(
      command.pose.z(), our_robot.theta, true);

    viz::DrawTrajectory(robot_id, maybe_trajectory.value());

    // TODO(jneiger): move this to a better spot
    Eigen::Vector2d robot{world.our_robots.at(robot_id).value().pos.x(), world.our_robots.at(
        robot_id).value().pos.y()};
    motion_command.kick = world.get_unique_ball().has_value() &&
      (world.get_unique_ball().value().pos - robot).norm() < 0.1;
    motion_command.kick_speed = 5;  // m/s
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
