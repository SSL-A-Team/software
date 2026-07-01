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

  if (prev_state_ != State::Pivot){
    if (prev_state_ == State::Capture) {
        if (!capture_.isDone()) {
          getPlayInfo()["State"] = "Capture";
          return Capture(world, robot);
        }
      }

      if (!robot.breakbeam_ball_detected_filtered) {
        prev_state_ = State::Capture;
        getPlayInfo()["State"] = "Capture";
        return Capture(world, robot);
      }
  }

  const auto robot_to_target = target_point_ - robot.pos;
  const auto robot_to_target_angle = std::atan2(robot_to_target.y(), robot_to_target.x());
  if (abs(angles::shortest_angular_distance(robot.theta, robot_to_target_angle)) > 0.05) {
    if (prev_state_ != State::Pivot) {
      prev_state_ = State::Pivot;
    }
    getPlayInfo()["State"] = "Pivot";
    return Pivot(world, robot);
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
  return KickBall(robot);
}

RobotCommand PivotKick::Capture(
  const World & world,
  const Robot & robot)
{
  RobotCommand capture_result = capture_.runFrame(world, robot);
  ForwardPlayInfo(capture_);
  return capture_result;
}

RobotCommand PivotKick::Pivot(
  const World & world,
  const Robot & robot)
{
  (void)robot;
  (void)world;
  // const auto robot_to_target = target_point_ - robot.pos;
  // const auto robot_to_target_angle = std::atan2(robot_to_target.y(), robot_to_target.x());

  // const auto ball_to_target = target_point_ - world.ball.pos;
  // const auto ball_to_target_angle = std::atan2(ball_to_target.y(), ball_to_target.x());

  // motion::intents::PivotHeading intent;
  motion::intents::PivotPoint intent;
  intent.target_x = target_point_.x();
  intent.target_y = target_point_.y();

  // intent.radius = 0.9 * (kRobotRadius + kBallRadius);
  intent.radius = 0.2;


  // intent.target_heading = robot_to_target_angle;
  // intent.target_heading = ball_to_target_angle;
  // intent.inset_angle = M_PI / 2.0;
  intent.inset_angle = 0.0;

  intent.limits.angular_velocity = 1.5;
  intent.limits.angular_acceleration = 4.0;

  RobotCommand command;
  command.motion_intent = intent;
  command.dribbler_setpoint = 2.0*kDefaultDribblerSetpoint;

  return command;
}

RobotCommand PivotKick::KickBall(const Robot & robot)
{
  motion::intents::PositionFacing intent;

  intent.planner_options.avoid_ball = false;
  intent.face_target = target_point_;
  intent.position = robot.pos;

  RobotCommand command;

  command.motion_intent = intent;

  command.dribbler_setpoint = kDefaultDribblerSetpoint;

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
