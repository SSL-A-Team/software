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


#ifndef SKILLS__PIVOT_KICK_HPP_
#define SKILLS__PIVOT_KICK_HPP_

#include <ateam_common/robot_constants.hpp>
#include "kick_skill.hpp"
#include "core/types.hpp"
#include "core/types/robot_command.hpp"
#include "skills/capture.hpp"

namespace ateam_kenobi::skills
{

class PivotKick : public KickSkill
{
public:
  enum class KickType
  {
    Kick,
    Chip
  };

  explicit PivotKick(
    stp::Options stp_options,
    KickSkill::WaitType wait_type = KickSkill::WaitType::KickWhenReady);

  void Reset() override
  {
    KickSkill::Reset();
    capture_.Reset();
    prev_state_ = State::Capture;
    done_ = false;
  }

  void SetTargetPoint(ateam_geometry::Point point)
  {
    target_point_ = point;
  }

  ateam_geometry::Point GetAssignmentPoint(const World & world);

  RobotCommand RunFrame(const World & world, const Robot & robot);

  bool IsDone() const
  {
    return done_;
  }

  void SetCaptureSpeed(double speed)
  {
    capture_.SetCaptureSpeed(speed);
  }

  void SetPivotSpeed(double speed)
  {
    pivot_speed_ = speed;
  }

  bool IsReady() const override
  {
    return prev_state_ == State::Pivot;
  }

private:
  ateam_geometry::Point target_point_;
  skills::Capture capture_;
  bool done_ = false;
  double pivot_speed_ = 2.0;  // rad/s
  double pivot_accel_ = 1.5;  // rad/s^2

  enum class State
  {
    Capture,
    Pivot,
    KickBall
  };
  State prev_state_ = State::Capture;

  RobotCommand Capture(const World & world, const Robot & robot);

  RobotCommand Pivot(const Robot & robot);

  RobotCommand KickBall();
};

}  // namespace ateam_kenobi::skills

#endif  // SKILLS__PIVOT_KICK_HPP_
