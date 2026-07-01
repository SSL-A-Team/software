// Copyright 2024 A Team
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


#include "capture.hpp"
#include <angles/angles.h>
#include <algorithm>
#include <vector>
#include <ateam_geometry/normalize.hpp>
#include "core/play_helpers/available_robots.hpp"

namespace ateam_kenobi::skills
{

Capture::Capture(stp::Options stp_options)
: stp::Skill(stp_options)
{}

void Capture::Reset()
{
  done_ = false;
  ball_detected_filter_ = 0;
}

RobotCommand Capture::runFrame(const World & world, const Robot & robot)
{
  chooseState(world, robot);

  switch (state_) {
    case State::MoveToBall:
      return runMoveToBall(world, robot);
    case State::Capture:
      return runCapture(world, robot);
    default:
      std::cerr << "Unhandled state in Capture!\n";
      return RobotCommand{};
  }
}

void Capture::chooseState(const World & world, const Robot & robot)
{
  if(state_ == State::Capture && !world.ball.visible) {
    state_ = State::Capture;
  } else if (ateam_geometry::norm(world.ball.pos - robot.pos) < approach_radius_ + kRobotRadius) {
    state_ = State::Capture;
  } else {
    state_ = State::MoveToBall;
  }
}

RobotCommand Capture::runMoveToBall(
  const World & world,
  const Robot & robot)
{
  const auto robot_to_ball_vector = world.ball.pos - robot.pos;

  motion::intents::PositionFacing intent;
  intent.position = world.ball.pos - approach_radius_ * ateam_geometry::normalize(robot_to_ball_vector);
  intent.face_target = world.ball.pos;
  intent.planner_options.avoid_ball = false;

  // const auto distance_to_ball = CGAL::approximate_sqrt(CGAL::squared_distance(robot.pos,
  //     world.ball.pos));

  // const auto decel_distance = distance_to_ball - approach_radius_;

  // const auto max_decel_vel = std::sqrt((2.0 * decel_limit_ * decel_distance) +
  //     (capture_speed_ * capture_speed_));

  // intent.limits.linear_velocity = std::min(max_decel_vel, max_speed_);

  intent.limits.linear_velocity = max_speed_;
  intent.limits.linear_acceleration = decel_limit_;
  RobotCommand command;
  command.motion_intent = intent;

  return command;
}

RobotCommand Capture::runCapture(const World & world, const Robot & robot)
{
  // TODO(chachmu): Should we filter this over a few frames to make sure we have the ball settled?
  if (robot.breakbeam_ball_detected) {
    ball_detected_filter_ += 1;
    if (ball_detected_filter_ >= 30) {
      done_ = true;
    }
  } else {
    ball_detected_filter_ -= 2;
    if (ball_detected_filter_ < 0) {
      ball_detected_filter_ = 0;
    }
  }

  RobotCommand command;

  if(world.ball.visible) {
    motion::intents::PositionFacing intent;
    intent.planner_options.avoid_ball = false;
    intent.position = world.ball.pos;
    intent.face_target = world.ball.pos;
    intent.limits.linear_velocity = capture_speed_;
    intent.limits.linear_acceleration = 1.5;
    command.motion_intent = intent;
  } else {
    motion::intents::Position intent;
    intent.planner_options.avoid_ball = false;
    intent.position = world.ball.pos;
    intent.heading = robot.theta;
    intent.limits.linear_velocity = capture_speed_;
    intent.limits.linear_acceleration = 1.5;
    command.motion_intent = intent;
  }

  command.dribbler_setpoint = 2 * kDefaultDribblerSetpoint;

  return command;
}
}  // namespace ateam_kenobi::skills
