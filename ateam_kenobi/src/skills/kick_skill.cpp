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

#include "kick_skill.hpp"

namespace ateam_kenobi::skills
{

KickSkill::KickSkill(stp::Options stp_options, WaitType wait_type)
: stp::Skill(stp_options),
  wait_type_(wait_type)
{
  if (wait_type_ == WaitType::WaitToKick) {
    kicking_allowed_ = false;
  }
}

void KickSkill::Reset()
{
  if (wait_type_ == WaitType::WaitToKick) {
    kicking_allowed_ = false;
  }
}

void KickSkill::AllowKicking()
{
  kicking_allowed_ = true;
}

void KickSkill::DisallowKicking()
{
  if (wait_type_ == WaitType::WaitToKick) {
    kicking_allowed_ = false;
  }
}

void KickSkill::SetKickSpeed(double speed)
{
  // TODO(barulicm) Should we limit kick speeds here?
  kick_speed_ = speed;
}

double KickSkill::GetKickSpeed() const
{
  return kick_speed_;
}

bool KickSkill::IsAllowedToKick() const
{
  return kicking_allowed_;
}

}  // namespace ateam_kenobi::skills
