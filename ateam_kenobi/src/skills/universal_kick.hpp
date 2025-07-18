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

#ifndef SKILLS__UNIVERSAL_KICK_HPP_
#define SKILLS__UNIVERSAL_KICK_HPP_

#include "kick_skill.hpp"
#include "pivot_kick.hpp"
#include "line_kick.hpp"

namespace ateam_kenobi::skills
{

class UniversalKick : public KickSkill
{
public:
  enum class KickType
  {
    Unset,
    Pivot,
    Line
  };

  explicit UniversalKick(stp::Options stp_options, WaitType wait_type = WaitType::KickWhenReady);

  void Reset() override;

  bool IsReady() const override;

  void SetTargetPoint(ateam_geometry::Point point);

  ateam_geometry::Point GetAssignmentPoint(const World & world);

  bool IsDone() const;

  void SetUseDefaultObstacles(bool use_obstacles);

  void SetKickChip(KickSkill::KickChip kc);

  void SetPreferredKickType(KickType type);

  ateam_msgs::msg::RobotMotionCommand RunFrame(const World & world, const Robot & robot);

  /*
   * Pivot-only functions
   */


  void SetCaptureSpeed(double speed);

  void SetPivotSpeed(double speed);

  /*
   * Line-only functions
   */

  void SetPreKickOffset(double val);

private:
  static constexpr bool kPivotAllowed = true;
  static constexpr bool kLineAllowed = true;

  PivotKick pivot_kick_;
  LineKick line_kick_;
  KickType preferred_type_ = KickType::Unset;
  KickType last_used_ = KickType::Unset;

  KickType ChooseType() const;

};

}  // namespace ateam_kenobi::skills

#endif  // SKILLS__UNIVERSAL_KICK_HPP_
