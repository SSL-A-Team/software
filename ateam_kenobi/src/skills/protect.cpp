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


#include "protect.hpp"
#include <angles/angles.h>
#include <vector>
#include <ateam_msgs/msg/robot_motion_command.hpp>
#include <ateam_geometry/normalize.hpp>
#include "core/play_helpers/available_robots.hpp"

namespace ateam_kenobi::skills
{

Protect::Protect(stp::Options stp_options)
: Skill(stp_options),
  capture_(createChild<skills::Capture>("Capture"))
{
}

ateam_geometry::Point Protect::GetAssignmentPoint(const World & world)
{
  // Could use point between opponent and ball or something
  return world.ball.pos;
}

RobotCommand Protect::RunFrame(const World & world, const Robot & robot)
{
  // Default to blocking the path to our goal
  auto block_point = ateam_geometry::Point(-world.field.field_length / 2.0, 0.0);

  auto visible_opponents = play_helpers::getVisibleRobots(world.their_robots);
  std::ranges::sort(
    visible_opponents, [&world](const Robot & r1, const Robot & r2) {
      const auto r1_dist = ateam_geometry::norm(world.ball.pos, r1.pos);
      const auto r2_dist = ateam_geometry::norm(world.ball.pos, r2.pos);
      return r1_dist < r2_dist;
    });

  if (!visible_opponents.empty()) {
    const auto & opponent = visible_opponents[0];

    // This should probably go in a helper somewhere
    const double projection_timestep = 0.3;  // s
    const double max_bot_proj_vel = 1.0;  // m/s
    const double clamped_bot_vel_mag = std::clamp(ateam_geometry::norm(opponent.vel), 0.0,
        max_bot_proj_vel);
    const ateam_geometry::Vector clamped_bot_vel = clamped_bot_vel_mag *
      ateam_geometry::normalize(opponent.vel);
    const auto projected_bot_pos = opponent.pos + projection_timestep * clamped_bot_vel;

    block_point = projected_bot_pos;
  }

  if (robot.breakbeam_ball_detected_filtered) {
    state_ = State::Pivot;
  } else {
    state_ = State::Capture;
  }

  if (state_ == State::Capture) {
    getOverlays().drawLine("protect_line", {world.ball.pos, block_point}, "#FF00007F");
  } else {
    getOverlays().drawLine("protect_line", {world.ball.pos, block_point}, "#FFFF007F");
  }

  switch(state_) {
    case State::Pivot:
      return Pivot(world, robot, block_point);
      break;
    case State::Capture:
    default:
      return Capture(world, robot);
      break;
  }
}

RobotCommand Protect::Capture(
  const World & world,
  const Robot & robot)
{
  RobotCommand capture_result = capture_.runFrame(world, robot);
  ForwardPlayInfo(capture_);
  return capture_result;
}

RobotCommand Protect::Pivot(
  const World & world,
  const Robot & robot,
  const ateam_geometry::Point & block_point)
{
  (void)robot;
  (void)world;

  const auto ball_to_block = block_point - world.ball.pos;
  const auto block_angle = ateam_geometry::ToHeading(-ball_to_block);

  motion::intents::PivotHeading intent;
  intent.radius = 0.02 + kRobotRadius + kBallRadius;
  intent.target_heading = block_angle;
  intent.inset_angle = M_PI / 2.0;

  intent.limits.angular_velocity = 2.0;
  intent.limits.angular_acceleration = 2.0;

  RobotCommand command;
  command.motion_intent = intent;
  command.dribbler_setpoint = kDefaultDribblerSetpoint;

  return command;
}

}  // namespace ateam_kenobi::skills
