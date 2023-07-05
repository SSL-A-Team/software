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

#include <ateam_msgs/msg/robot_motion_command.hpp>
#include "robot_assignment.hpp"


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

ateam_msgs::msg::RobotMotionCommand line_kick_command(const World & world, Robot current_robot, ateam_geometry::Point target)
{

  Eigen::Vector3d current, current_vel, target, target_vel;
  current.x() = current_robot.pos.x();
  current.y() = current_robot.pos.y();
  current.z() = current_robot.theta;
  current_vel.x() = current_robot.vel.x();
  current_vel.y() = current_robot.vel.y();
  current_vel.z() = current_robot.omega;

  const auto & ball = world.ball;
  if (ball.vel.norm() > 0.05) {
    target.x() = current_robot.pos.x();
    target.y() = current_robot.pos.y();
    target.z() = current_robot.theta;
    target_vel.x() = 0;
    target_vel.y() = 0;
    target_vel.z() = 0;
  } else {
    // robot to ball check to not collide
    ateam_geometry::Point ball_pos = ball.pos;
    ateam_geometry::Point current_robot_pos = current_robot.pos;
    ateam_geometry::Point target_goal = ateam_geometry::Point{-6, 0};
    ateam_geometry::Point target_setup_pos = ball_pos + 0.4 * normalize(ball_pos - target_goal);

    ateam_geometry::Point robot_to_goal = target_goal - current_robot_pos;
    double robot_to_goal_angle = ateam_common::geometry::VectorToAngle(robot_to_goal);
    double angle_diff = angles::shortest_angular_distance(
      current_robot.theta, robot_to_goal_angle);

    ateam_geometry::Point robot_to_ball = ball_pos - current_robot_pos;
    ateam_geometry::Point ball_to_goal = target_goal - ball_pos;

    // Aligned means
    //  * Ball is directly between the robot and goal
    //  * Angle difference between heading and robot to goal is low
    //  * Robot is almost stopped
    bool is_aligned = ateam_common::geometry::IsVectorAligned(
      robot_to_goal, robot_to_ball,
      0.01);
    is_aligned &= std::abs(angle_diff) < 0.05;
    is_aligned &= current_robot.vel.norm() < 0.5;

    if (!is_aligned) {
      double cosine = CGAL::scalar_product((target_setup_pos - current_robot_pos),
        (ball_pos - current_robot_pos)) /
        (norm(target_setup_pos, current_robot_pos) *
        norm(target_setup_pos, current_robot_pos));
      cosine = std::max(std::min(cosine, 1.0), 0.0);
      ateam_geometry::Point ball_projected_on_move_line = cosine *
        (target_setup_pos - current_robot_pos) + current_robot_pos;
      double dist = norm(ball_projected_on_move_line, ball_pos);
      if (dist < 0.1) {
        target.x() = target_setup_pos.x();
        target.y() = target_setup_pos.y() - 1;
        target.z() = atan2(ball_to_goal.y(), ball_to_goal.x());
        target_vel.x() = 0;
        target_vel.y() = 0;
        target_vel.z() = 0;
      } else {
        target.x() = target_setup_pos.x();
        target.y() = target_setup_pos.y();
        target.z() = atan2(ball_to_goal.y(), ball_to_goal.x());
        target_vel.x() = 0;
        target_vel.y() = 0;
        target_vel.z() = 0;
      }
    } else {
      // Kick
      target.x() = ball_pos.x();
      target.y() = ball_pos.y();
      target.z() = atan2(ball_to_goal.y(), ball_to_goal.x());
      target_vel.x() = 0;
      target_vel.y() = 0;
      target_vel.z() = 0;
    }
  }

  Eigen::Vector3d max_vel{3, 3, 1};  // TODO(jneiger): Set as params
  Eigen::Vector3d max_accel{1, 1, 1};
  double dt = 0.01;  // TODO(jneiger): Feed this down from above
  Trajectory trajectory = TrapezoidalMotionProfile::Generate3d(
    current, current_vel, target,
    target_vel, max_vel, max_accel,
    dt, world.current_time + world.immutable_duration);

  plan.trajectory = trajectory;
}


//   Eigen::Vector3d current, current_vel, target, target_vel;
//   current.x() = current_robot.pos.x();
//   current.y() = current_robot.pos.y();
//   current.z() = current_robot.theta;
//   current_vel.x() = current_robot.vel.x();
//   current_vel.y() = current_robot.vel.y();
//   current_vel.z() = current_robot.omega;

//   const auto & ball = world.get_unique_ball();
//   if (!ball.has_value() || ball.vel.norm() > 0.05) {
//     target.x() = current_robot.pos.x();
//     target.y() = current_robot.pos.y();
//     target.z() = current_robot.theta;
//     target_vel.x() = 0;
//     target_vel.y() = 0;
//     target_vel.z() = 0;
//   } else {
//     // robot to ball check to not collide
//     ateam_geometry::Point ball_pos = ball.pos;
//     ateam_geometry::Point current_robot_pos = current_robot.pos;
//     ateam_geometry::Point target_goal = ateam_geometry::Point{-6, 0};
//     ateam_geometry::Point target_setup_pos = ball_pos + 0.4 * (ball_pos - target_goal).normalized();

//     ateam_geometry::Point robot_to_goal = target_goal - current_robot_pos;
//     double robot_to_goal_angle = ateam_common::geometry::VectorToAngle(robot_to_goal);
//     double angle_diff = angles::shortest_angular_distance(
//       current_robot.theta, robot_to_goal_angle);

//     ateam_geometry::Point robot_to_ball = ball_pos - current_robot_pos;
//     ateam_geometry::Point ball_to_goal = target_goal - ball_pos;

//     // Aligned means
//     //  * Ball is directly between the robot and goal
//     //  * Angle difference between heading and robot to goal is low
//     //  * Robot is almost stopped
//     bool is_aligned = ateam_common::geometry::IsVectorAligned(
//       robot_to_goal, robot_to_ball,
//       0.01);
//     is_aligned &= std::abs(angle_diff) < 0.05;
//     is_aligned &= current_robot.vel.norm() < 0.5;

//     if (!is_aligned) {
//       double projection = (target_setup_pos - current_robot_pos).dot(
//         ball_pos - current_robot_pos) /
//         ((target_setup_pos - current_robot_pos).norm() *
//         (target_setup_pos - current_robot_pos).norm());
//       projection = std::max(std::min(projection, 1.0), 0.0);
//       ateam_geometry::Point ball_projected_on_move_line = projection *
//         (target_setup_pos - current_robot_pos) + current_robot_pos;
//       double dist = (ball_projected_on_move_line - ball_pos).norm();
//       if (dist < 0.1) {
//         target.x() = target_setup_pos.x();
//         target.y() = target_setup_pos.y() - 1;
//         target.z() = atan2(ball_to_goal.y(), ball_to_goal.x());
//         target_vel.x() = 0;
//         target_vel.y() = 0;
//         target_vel.z() = 0;
//       } else {
//         target.x() = target_setup_pos.x();
//         target.y() = target_setup_pos.y();
//         target.z() = atan2(ball_to_goal.y(), ball_to_goal.x());
//         target_vel.x() = 0;
//         target_vel.y() = 0;
//         target_vel.z() = 0;
//       }
//     } else {
//       // Kick
//       target.x() = ball_pos.x();
//       target.y() = ball_pos.y();
//       target.z() = atan2(ball_to_goal.y(), ball_to_goal.x());
//       target_vel.x() = 0;
//       target_vel.y() = 0;
//       target_vel.z() = 0;
//     }
//   }

//   Eigen::Vector3d max_vel{3, 3, 1};  // TODO(jneiger): Set as params
//   Eigen::Vector3d max_accel{1, 1, 1};
//   double dt = 0.01;  // TODO(jneiger): Feed this down from above
//   Trajectory trajectory = TrapezoidalMotionProfile::Generate3d(
//     current, current_vel, target,
//     target_vel, max_vel, max_accel,
//     dt, world.current_time + world.immutable_duration);

//   plan.trajectory = trajectory;
// }

} // namespace ateam_kenobi::skills

#endif // SKILLS__KICK_HPP_
