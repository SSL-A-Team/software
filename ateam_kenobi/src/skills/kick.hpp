// Copyright 2023 A Team
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

#ifndef SKILLS__KICK_HPP_
#define SKILLS__KICK_HPP_

#include <ateam_geometry/normalize.hpp>
#include <ateam_geometry/types.hpp>
#include <ateam_common/angle.hpp>

#include <ateam_msgs/msg/robot_motion_command.hpp>
#include "robot_assignment.hpp"

#include <angles/angles.h>


#include "path_planning/path_planner.hpp"
#include "motion/motion_controller.hpp"

namespace ateam_kenobi::skills
{
ateam_msgs::msg::RobotMotionCommand send_kick_command()
{
  auto kick_command = ateam_msgs::msg::RobotMotionCommand{};
  kick_command.kick = true;
  // Set kick speed to 5 m/s. Current max is 5.5 m/s (will
  // be clamped if above).
  kick_command.kick_speed = 5;
  return kick_command;

}

std::vector<ateam_geometry::Point> line_kick_command(const World & world, Robot current_robot, ateam_geometry::Point target_goal)
{
  // This is bad make these a persistent state or quit
  path_planning::PathPlanner path_planner_;
  MotionController motion_controller_;

  double current_theta, current_vel_theta, target_theta, target_vel_theta;
  ateam_geometry::Point current (current_robot.pos.x(), current_robot.pos.y());
  current_theta = current_robot.theta;
  ateam_geometry::Point current_vel (current_robot.vel.x(), current_robot.vel.y());
  current_vel_theta = current_robot.omega;

  ateam_geometry::Point target, target_vel;
  const auto & ball = world.ball;
  if (ateam_geometry::norm(ball.vel) > 0.05) {
    target = ateam_geometry::Point(current_robot.pos.x(), current_robot.pos.y());
    target_theta = current_robot.theta;
    target_vel = ateam_geometry::Point(0,0);
    target_vel_theta = 0;
  } else {
    // robot to ball check to not collide
    ateam_geometry::Point ball_pos = ball.pos;
    ateam_geometry::Point current_robot_pos = current_robot.pos;
    ateam_geometry::Point target_setup_pos = ball_pos + 0.4 * ateam_geometry::normalize(ball_pos - target_goal);

    ateam_geometry::Vector robot_to_goal = target_goal - current_robot_pos;
    double robot_to_goal_angle = ateam_common::geometry::VectorToAngle(robot_to_goal);
    double angle_diff = angles::shortest_angular_distance(
      current_robot.theta, robot_to_goal_angle);

    ateam_geometry::Vector robot_to_ball = ball_pos - current_robot_pos;
    ateam_geometry::Vector ball_to_goal = target_goal - ball_pos;

    // Aligned means
    //  * Ball is directly between the robot and goal
    //  * Angle difference between heading and robot to goal is low
    //  * Robot is almost stopped
    bool is_aligned = ateam_common::geometry::IsVectorAligned(
      robot_to_goal, robot_to_ball,
      0.01);
    is_aligned &= std::abs(angle_diff) < 0.05;
    is_aligned &= ateam_geometry::norm(current_robot.vel) < 0.5;

    if (!is_aligned) {
      double cosine = CGAL::scalar_product((target_setup_pos - current_robot_pos),
        (ball_pos - current_robot_pos)) /
        (ateam_geometry::norm(target_setup_pos, current_robot_pos) *
        ateam_geometry::norm(target_setup_pos, current_robot_pos));
      cosine = std::max(std::min(cosine, 1.0), 0.0);
      ateam_geometry::Vector ball_projected_on_move_line = cosine *
        (target_setup_pos - current_robot_pos) + ateam_geometry::Vector(current_robot_pos.x(), current_robot_pos.y());
      double dist = ateam_geometry::norm(ateam_geometry::Point(ball_projected_on_move_line.x(),
        ball_projected_on_move_line.y()), ball_pos);
      if (dist < 0.1) {
        target = ateam_geometry::Point(target_setup_pos.x(), target_setup_pos.y() - 1);
        target_theta = atan2(ball_to_goal.y(), ball_to_goal.x());
        target_vel = ateam_geometry::Point(0,0);
        target_vel_theta = 0;
      } else {
        target = ateam_geometry::Point(target_setup_pos.x(), target_setup_pos.y());
        target_theta = atan2(ball_to_goal.y(), ball_to_goal.x());
        target_vel = ateam_geometry::Point(0,0);
        target_vel_theta = 0;
      }

    } else {
      // Kick
      target = ateam_geometry::Point(ball_pos.x(), ball_pos.y());
      target_theta = atan2(ball_to_goal.y(), ball_to_goal.x());
      target_vel = ateam_geometry::Point(0,0);
      target_vel_theta = 0;
    }
  }

  const auto kick_path = path_planner_.getPath(current_robot.pos,
    target, world, {});
  return kick_path;
}
} // namespace ateam_kenobi::skills

#endif // SKILLS__KICK_HPP_
