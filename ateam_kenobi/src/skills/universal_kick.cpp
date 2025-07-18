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

#include "universal_kick.hpp"

namespace ateam_kenobi::skills
{

UniversalKick::UniversalKick(stp::Options stp_options, WaitType wait_type)
: KickSkill(stp_options, wait_type),
  pivot_kick_(createChild<skills::PivotKick>("PivotKick", wait_type)),
  line_kick_(createChild<skills::LineKick>("LineKick", wait_type))
{
}

void UniversalKick::Reset()
{
  pivot_kick_.Reset();
  line_kick_.Reset();
}

bool UniversalKick::IsReady() const
{
  switch(last_used_) {
    case KickType::Pivot:
      return pivot_kick_.IsReady();
    case KickType::Line:
      return line_kick_.IsReady();
    case KickType::Unset:
    default:
      return false;
  }
}

void UniversalKick::SetTargetPoint(ateam_geometry::Point point)
{
  pivot_kick_.SetTargetPoint(point);
  line_kick_.SetTargetPoint(point);
}

ateam_geometry::Point UniversalKick::GetAssignmentPoint(const World & world)
{
  switch(ChooseType()) {
    case KickType::Pivot:
      return pivot_kick_.GetAssignmentPoint(world);
    case KickType::Line:
      return line_kick_.GetAssignmentPoint(world);
    case KickType::Unset:
    default:
      return ateam_geometry::Point{};
  }
}

bool UniversalKick::IsDone() const
{
  switch(last_used_) {
    case KickType::Pivot:
      return pivot_kick_.IsDone();
    case KickType::Line:
      return line_kick_.IsDone();
    case KickType::Unset:
    default:
      return false;
  }
}

void UniversalKick::SetUseDefaultObstacles(bool use_obstacles)
{
  pivot_kick_.SetUseDefaultObstacles(use_obstacles);
  line_kick_.SetUseDefaultObstacles(use_obstacles);
}

void UniversalKick::SetKickChip(KickSkill::KickChip kc)
{
  pivot_kick_.SetKickChip(kc);
  line_kick_.SetKickChip(kc);
}

void UniversalKick::SetPreferredKickType(KickType type)
{
  preferred_type_ = type;
}

ateam_msgs::msg::RobotMotionCommand UniversalKick::RunFrame(
  const World & world,
  const Robot & robot)
{
  if(IsAllowedToKick()) {
    pivot_kick_.AllowKicking();
    line_kick_.AllowKicking();
  } else {
    pivot_kick_.DisallowKicking();
    line_kick_.DisallowKicking();
  }
  pivot_kick_.SetKickSpeed(GetKickSpeed());
  line_kick_.SetKickSpeed(GetKickSpeed());
  pivot_kick_.SetKickChip(KickOrChip());
  line_kick_.SetKickChip(KickOrChip());
  switch(ChooseType()) {
    case KickType::Pivot:
      last_used_ = KickType::Pivot;
      return pivot_kick_.RunFrame(world, robot);
    case KickType::Line:
      last_used_ = KickType::Line;
      return line_kick_.RunFrame(world, robot);
    case KickType::Unset:
    default:
      last_used_ = KickType::Unset;
      return ateam_msgs::msg::RobotMotionCommand{};
  }
}

void UniversalKick::SetCaptureSpeed(double speed)
{
  pivot_kick_.SetCaptureSpeed(speed);
}

void UniversalKick::SetPivotSpeed(double speed)
{
  pivot_kick_.SetPivotSpeed(speed);
}

void UniversalKick::SetPreKickOffset(double val)
{
  line_kick_.setPreKickOffset(val);
}

UniversalKick::KickType UniversalKick::ChooseType() const
{
  if(preferred_type_ == KickType::Pivot && kPivotAllowed) {
    return KickType::Pivot;
  }
  if(preferred_type_ == KickType::Line && kLineAllowed) {
    return KickType::Line;
  }
  if(kPivotAllowed) {
    return KickType::Pivot;
  }
  if(kLineAllowed) {
    return KickType::Line;
  }
  return KickType::Unset;
}

}  // namespace ateam_kenobi::skills
