// Copyright 2026 A Team
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

#include "skills/dribble_kick.hpp"

namespace ateam_kenobi::skills
{

DribbleKick::DribbleKick(stp::Options stp_options)
: stp::Skill(stp_options),
  kick_(createChild<skills::LineKick>("kick"))
{
  kick_.SetKickSpeed(kDefaultKickSpeed);
}

void DribbleKick::Reset()
{
  kick_.Reset();
}

void DribbleKick::SetKickSpeed(double speed)
{
  kick_.SetKickSpeed(speed);
}

ateam_geometry::Point DribbleKick::GetAssignmentPoint(const World & world)
{
  return world.ball.pos;
}

RobotCommand DribbleKick::RunFrame(
  const World & world, const Robot & robot,
  const ateam_geometry::Direction & direction)
{
  if(kick_.IsDone()) {
    kick_.Reset();
  }
  kick_.SetTargetPoint(world.ball.pos + (direction.to_vector() * 2.0));
  RobotCommand command = kick_.RunFrame(world, robot);
  ForwardPlayInfo(kick_);
  return command;
}

}  // namespace ateam_kenobi::skills
