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


#include "pass.hpp"
#include <algorithm>

namespace ateam_kenobi::tactics
{

Pass::Pass(stp::Options stp_options)
: stp::Tactic(stp_options),
  receiver_(createChild<skills::PassReceiver>("receiver")),
  kick_(createChild<skills::LineKick>("kicker", skills::KickSkill::WaitType::WaitToKick))
{
}

void Pass::reset()
{
  receiver_.reset();
  kick_.Reset();
}

ateam_geometry::Point Pass::getKickerAssignmentPoint(const World & world)
{
  return kick_.GetAssignmentPoint(world);
}

ateam_geometry::Point Pass::getReceiverAssignmentPoint()
{
  return receiver_.getAssignmentPoint();
}

void Pass::runFrame(
  const World & world, const Robot & kicker_bot, const Robot & receiver_bot,
  ateam_msgs::msg::RobotMotionCommand & kicker_command,
  ateam_msgs::msg::RobotMotionCommand & receiver_command)
{
  receiver_command = receiver_.runFrame(world, receiver_bot);

  if (kick_.IsDone() && !receiver_.isDone() && ateam_geometry::norm(world.ball.vel) < 0.01) {
    kick_.Reset();
  }

  if (ateam_geometry::norm(receiver_bot.pos, target_) <= kReceiverPositionThreshold) {
    kick_.AllowKicking();
  } else {
    kick_.DisallowKicking();
  }

  if (speed_) {
    kick_.SetKickSpeed(*speed_);
  } else {
    kick_.SetKickSpeed(calculateDefaultKickSpeed(world));
  }
  kicker_command = kick_.RunFrame(world, kicker_bot);
}

bool Pass::isDone()
{
  return receiver_.isDone();
}

double Pass::calculateDefaultKickSpeed(const World & world)
{
  const auto distance = CGAL::approximate_sqrt(CGAL::squared_distance(world.ball.pos, target_));
  const auto ball_friction_acceleration = 0.5;
  const auto max_kick_speed = 5.0;
  const auto velocity_at_receiver = 0.5;
  const auto stop_at_receiver_velocity = std::sqrt(2.0 * ball_friction_acceleration * distance);
  return std::min(velocity_at_receiver + stop_at_receiver_velocity, max_kick_speed);
}

}  // namespace ateam_kenobi::tactics
