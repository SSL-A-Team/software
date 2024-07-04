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


#include "pivot_kick.hpp"
#include <angles/angles.h>
#include <vector>
#include <ateam_msgs/msg/robot_motion_command.hpp>
#include <ateam_geometry/normalize.hpp>

namespace ateam_kenobi::skills
{

PivotKick::PivotKick(stp::Options stp_options, KickSkill::WaitType wait_type)
: KickSkill(stp_options, wait_type),
  easy_move_to_(createChild<play_helpers::EasyMoveTo>("easy_move_to"))
{
}

ateam_geometry::Point PivotKick::GetAssignmentPoint(const World & world)
{
  return world.ball.pos;
}

ateam_msgs::msg::RobotMotionCommand PivotKick::RunFrame(const World & world, const Robot & robot)
{
  getOverlays().drawLine("PivotKick_line", {world.ball.pos, target_point_}, "#FFFF007F");

  bool breakbeam_ball_detected = robot.breakbeam_ball_detected;

  // I guess we could make a param in kenobi node that causes it to automatically
  // calculate simulated breakbeam for each bot when it is populating the robot objects
  bool use_sim_breakbeam = true;
  if (use_sim_breakbeam) {
    float hysteresis = 1.0;
    if (prev_state_ != State::Capture) {
      hysteresis = 2.0;
    }

    const auto robot_to_ball = world.ball.pos - robot.pos;
    const auto distance_to_ball = ateam_geometry::norm(robot.pos, world.ball.pos);
    const auto robot_to_ball_angle = std::atan2(robot_to_ball.y(), robot_to_ball.x());
    breakbeam_ball_detected = distance_to_ball < (0.08+kBallRadius) * hysteresis
      && angles::shortest_angular_distance(robot.theta, robot_to_ball_angle) < 0.15 * hysteresis;
  }

  if (!breakbeam_ball_detected) {
    if (prev_state_ != State::Capture) {
      easy_move_to_.reset();
      prev_state_ = State::Capture;
    }
    RCLCPP_INFO(getLogger(), "Capturing...");
    return Capture(world, robot);
  }

  const auto robot_to_target = target_point_ - robot.pos;
  const auto robot_to_target_angle = std::atan2(robot_to_target.y(), robot_to_target.x());
  if (abs(angles::shortest_angular_distance(robot.theta, robot_to_target_angle)) > 0.05) {
    if (prev_state_ != State::Pivot) {
      easy_move_to_.reset();
      prev_state_ = State::Pivot;
    }
    RCLCPP_INFO(getLogger(), "Pivoting...");
    return Pivot(robot);
  }

  if (prev_state_ != State::KickBall) {
    easy_move_to_.reset();
    prev_state_ = State::KickBall;
  }

  RCLCPP_INFO(getLogger(), "Kicking...");
  return KickBall(world, robot);
}

ateam_msgs::msg::RobotMotionCommand PivotKick::Capture(
  const World & world,
  const Robot & robot)
{
  path_planning::PlannerOptions planner_options;
  planner_options.avoid_ball = false;
  planner_options.use_default_obstacles = true;

  easy_move_to_.setPlannerOptions(planner_options);
  easy_move_to_.setTargetPosition(world.ball.pos);


  const auto distance_to_ball = ateam_geometry::norm(robot.pos, world.ball.pos);
  if (distance_to_ball > 0.5) {
    easy_move_to_.face_travel();

    // TODO(chachmu): figure out a better way to reset max velocity
    easy_move_to_.setMaxVelocity(2.0);
  } else {
    double velocity = (distance_to_ball * 0.5) + 0.1;
    easy_move_to_.setMaxVelocity(velocity);
    easy_move_to_.face_point(world.ball.pos);
  }

  auto command = easy_move_to_.runFrame(robot, world);
  command.dribbler_speed = 200;
  return command;
}

ateam_msgs::msg::RobotMotionCommand PivotKick::Pivot(const Robot & robot)
{
  /*
  const auto robot_to_target = target_point_ - robot.pos;
  const auto robot_to_target_angle = std::atan2(robot_to_target.y(), robot_to_target.x());
  angles::shortest_angular_distance(robot.theta, robot_to_target_angle);
  */

  ateam_msgs::msg::RobotMotionCommand command;
  command.twist.angular.z = 1.5;  // turn at 1.5 rad/s

  // rotate in a circle with diameter 0.0427 + 0.18 = 0.2227
  // circumference of 0.6996 meters in a full rotation.
  // Calculate m/rev * rev/s to get linear m/s
  double velocity = 0.6996 * (command.twist.angular.z / (2 * M_PI));

  command.twist.linear.x = std::sin(robot.theta) * velocity;
  command.twist.linear.y = -std::cos(robot.theta) * velocity;
  command.dribbler_speed = 200;
  return command;
}

ateam_msgs::msg::RobotMotionCommand PivotKick::KickBall(const World & world, const Robot & robot)
{
  easy_move_to_.setTargetPosition(robot.pos);
  easy_move_to_.face_point(target_point_);
  path_planning::PlannerOptions planner_options;
  planner_options.avoid_ball = false;
  planner_options.footprint_inflation = 0.0;
  planner_options.use_default_obstacles = false;
  easy_move_to_.setPlannerOptions(planner_options);
  auto command = easy_move_to_.runFrame(robot, world);
  command.dribbler_speed = 500;
  command.kick = ateam_msgs::msg::RobotMotionCommand::KICK_ON_TOUCH;
  command.kick_speed = IsAllowedToKick() ? GetKickSpeed() : 0.0;
  return command;
}
}  // namespace ateam_kenobi::skills
