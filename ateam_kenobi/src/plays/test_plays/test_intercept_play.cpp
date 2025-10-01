// Copyright 2025 A Team
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

#include "test_intercept_play.hpp"
#include "core/types.hpp"
#include "core/play_helpers/available_robots.hpp"
#include "core/play_helpers/robot_assignment.hpp"

namespace ateam_kenobi::plays
{
TestInterceptPlay::TestInterceptPlay(stp::Options stp_options)
: stp::Play(kPlayName, stp_options),
  capture_skill_(createChild<skills::Capture>("capture")),
  kick_skill_(createChild<skills::PivotKick>("kick"))
{
}

void TestInterceptPlay::enter()
{
  ball_has_been_sensed_ = false;
  ball_has_been_kicked_ = false;
  capture_skill_.Reset();
  kick_skill_.Reset();
}

std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> TestInterceptPlay::runFrame(
  const World & world)
{
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> maybe_motion_commands;

  // pass_tactic_.setTarget(targets_[target_ind_]);

  const auto available_robots = play_helpers::getAvailableRobots(world);
  if (available_robots.size() >= 2) {
    const auto receiver = available_robots[0];
    const auto kicker = available_robots[1];

    getPlayInfo()["kicker_id"] = kicker.id;
    getPlayInfo()["receiver_id"] = receiver.id;

    auto & kicker_command = maybe_motion_commands.at(kicker.id).emplace();
    auto & receiver_command = maybe_motion_commands.at(receiver.id).emplace();

    const double kick_speed = 1.5;
    kick_skill_.SetTargetPoint(receiver.pos);
    kick_skill_.SetKickSpeed(kick_speed);

    if (capture_skill_.isDone()) {
      getPlayInfo()["result"] = "CAPTURE IS COMPLETED";
      return maybe_motion_commands;
    }

    const double ball_speed = ateam_geometry::norm(world.ball.vel);

    if (!ball_has_been_sensed_ && kicker.breakbeam_ball_detected) {
      ball_has_been_sensed_ = true;
    }

    if (ball_has_been_sensed_ && !ball_has_been_kicked_ &&
      ball_speed > 0.3 * kick_speed)
    {
      ball_has_been_kicked_ = true;
    }

    if (ball_has_been_kicked_) {
      ForwardPlayInfo(capture_skill_);
      receiver_command = capture_skill_.runFrame(world, receiver);
    } else {
      getPlayInfo()["ball sensed"] = ball_has_been_sensed_;
      getPlayInfo()["ball speed"] = ball_speed;

      kicker_command = kick_skill_.RunFrame(world, kicker);

      // Face the ball
      const double target_angle = atan2(
        world.ball.pos.y() - receiver.pos.y(),
        world.ball.pos.x() - receiver.pos.x());
      double t_error = angles::shortest_angular_distance(receiver.theta, target_angle);
      receiver_command.twist.angular.z = 1.5 * t_error;
    }
  }

  return maybe_motion_commands;
}
}  // namespace ateam_kenobi::plays
