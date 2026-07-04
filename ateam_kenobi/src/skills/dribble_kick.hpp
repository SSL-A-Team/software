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


#ifndef SKILLS__DRIBBLE_KICK_HPP_
#define SKILLS__DRIBBLE_KICK_HPP_

#include <ateam_common/robot_constants.hpp>
#include "core/stp/skill.hpp"
#include "core/types/state_types.hpp"
#include "core/types/robot_command.hpp"
#include "skills/line_kick.hpp"

namespace ateam_kenobi::skills
{

class DribbleKick : public stp::Skill
{
public:
  static constexpr double kDefaultKickSpeed = 0.2;

  explicit DribbleKick(stp::Options stp_options);

  void Reset();

  void SetKickSpeed(double speed);

  ateam_geometry::Point GetAssignmentPoint(const World & world);

  RobotCommand RunFrame(const World & world, const Robot & robot, const ateam_geometry::Direction & direction);

private:
  skills::LineKick kick_;
};

}  // namespace ateam_kenobi::skills

#endif  // SKILLS__DRIBBLE_KICK_HPP_
