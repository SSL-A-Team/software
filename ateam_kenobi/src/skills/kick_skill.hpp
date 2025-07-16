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

#ifndef SKILLS__KICK_SKILL_HPP_
#define SKILLS__KICK_SKILL_HPP_

#include "core/stp/skill.hpp"

namespace ateam_kenobi::skills
{

class KickSkill : public stp::Skill
{
public:
  /**
   * Controls whether the skill kicks automatically when it's ready or waits for
   * @c AllowKicking to be called.
   */
  enum class WaitType
  {
    KickWhenReady,
    WaitToKick
  };

  explicit KickSkill(stp::Options stp_options, WaitType wait_type = WaitType::KickWhenReady);

  virtual ~KickSkill() = default;

  virtual void Reset();

  /**
   * Allows the skill to kick at its discretion.
   * @note Only applies when wait type is @c WaitToKick
   */
  void AllowKicking();

  /**
   * Prevents the skill from kicking until @c AllowKicking is called.
   * @note Only applies when wait type is @c WaitToKick
   */
  void DisallowKicking();

  void SetKickSpeed(double speed);

  double GetKickSpeed() const;

protected:
  bool IsAllowedToKick() const;

private:
  WaitType wait_type_ = WaitType::KickWhenReady;
  bool kicking_allowed_ = true;
  double kick_speed_ = 4.0;
};

}  // namespace ateam_kenobi::skills

#endif  // SKILLS__KICK_SKILL_HPP_
