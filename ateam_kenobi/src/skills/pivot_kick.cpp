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
  capture_(createChild<skills::Capture>("Capture"))
{
}

ateam_geometry::Point PivotKick::GetAssignmentPoint(const World & world)
{
  return world.ball.pos;
}

RobotCommand PivotKick::RunFrame(const World & world, const Robot & robot)
{
  getOverlays().drawLine("PivotKick_line", {world.ball.pos, target_point_}, "#FFFF007F");

  if (done_) {
    getPlayInfo()["State"] = "Done";
    return RobotCommand{};
  }

  if (prev_state_ == State::Capture) {
    if (!capture_.isDone()) {
      getPlayInfo()["State"] = "Capture";
      return Capture(world, robot);
    }
  }

  if (!robot.breakbeam_ball_detected) {
    prev_state_ = State::Capture;
    getPlayInfo()["State"] = "Capture";
    return Capture(world, robot);
  }

  const auto robot_to_target = target_point_ - robot.pos;
  const auto robot_to_target_angle = std::atan2(robot_to_target.y(), robot_to_target.x());
  if (abs(angles::shortest_angular_distance(robot.theta, robot_to_target_angle)) > 0.05) {
    if (prev_state_ != State::Pivot) {
      prev_state_ = State::Pivot;
    }
    getPlayInfo()["State"] = "Pivot";
    return Pivot(robot);
  }

  if (prev_state_ != State::KickBall) {
    prev_state_ = State::KickBall;
  }

  if (ateam_geometry::norm(world.ball.vel) > 0.1 * GetKickSpeed()) {
    done_ = true;
    getPlayInfo()["State"] = "Done";
    return RobotCommand{};
  }

  getPlayInfo()["State"] = "Kick";
  return KickBall();
}

RobotCommand PivotKick::Capture(
  const World & world,
  const Robot & robot)
{
  return capture_.runFrame(world, robot);
}

RobotCommand PivotKick::Pivot(const Robot & robot)
{
  const auto robot_to_target = target_point_ - robot.pos;
  const auto robot_to_target_angle = std::atan2(robot_to_target.y(), robot_to_target.x());

  const auto angle_error = angles::shortest_angular_distance(robot.theta, robot_to_target_angle);

  RobotCommand command;

  const double vel = robot.prev_command_omega;
  const double dt = 0.01;

  double deceleration_to_reach_target = (vel * vel) / (2 * angle_error);

  // Cruise
  double trapezoidal_vel = std::copysign(pivot_speed_, angle_error);
  const double error_direction = std::copysign(1, angle_error);
  const double decel_direction = std::copysign(1, vel * angle_error);

  // Decelerate to target velocity
  if (decel_direction > 0 && abs(deceleration_to_reach_target) > pivot_accel_ * 0.95) {
    trapezoidal_vel = vel - (error_direction * deceleration_to_reach_target * dt);

  // Accelerate to speed
  } else if (abs(vel) < pivot_speed_) {
    trapezoidal_vel = vel + (error_direction * pivot_accel_ * dt);
  }

  const auto min_angular_vel = 1.0;
  if (abs(trapezoidal_vel) < min_angular_vel) {
    trapezoidal_vel = std::copysign(min_angular_vel, angle_error);
  }

  const auto angular_vel = std::clamp(trapezoidal_vel, -pivot_speed_, pivot_speed_);
  command.motion_intent.angular = motion::intents::angular::VelocityIntent{angular_vel};

  /* rotate in a circle with diameter 0.0427 + 0.18 = 0.2227 (This might be tunable to use 8cm for
   * real robots)
   * circumference of 0.6996 meters in a full rotation.
   * Calculate m/rev * rev/s to get linear m/s
   */
  // double diameter = kBallDiameter + kRobotDiameter;
  double diameter = (2 * .095) * 1.05;
  double circumference = M_PI * diameter;
  double velocity = circumference * (angular_vel / (2 * M_PI));

  command.motion_intent.linear = motion::intents::linear::VelocityIntent{
    ateam_geometry::Vector(0, -velocity),
    motion::intents::linear::Frame::Local};

  command.dribbler_speed = kDefaultDribblerSpeed;

  return command;
}

RobotCommand PivotKick::KickBall()
{
  RobotCommand command;

  command.motion_intent.angular = motion::intents::angular::FacingIntent{target_point_};

  command.dribbler_speed = kDefaultDribblerSpeed;

  command.kick_speed = GetKickSpeed();
  if (IsAllowedToKick()) {
    if(KickOrChip() == KickSkill::KickChip::Kick) {
      command.kick = KickState::KickOnTouch;
    } else {
      command.kick = KickState::ChipOnTouch;
    }
  }

  return command;
}
}  // namespace ateam_kenobi::skills
