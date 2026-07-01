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


#ifndef SKILLS__PROTECT_HPP_
#define SKILLS__PROTECT_HPP_

#include <ateam_common/robot_constants.hpp>
#include "core/types/state_types.hpp"
#include "core/types/robot_command.hpp"
#include "skills/capture.hpp"

namespace ateam_kenobi::skills
{

class Protect : public stp::Skill
{

public:
  explicit Protect(stp::Options stp_options);

  void Reset() {
    capture_.Reset();
  }

  ateam_geometry::Point GetAssignmentPoint(const World & world);

  RobotCommand RunFrame(const World & world, const Robot & robot);

private:
  enum class State
  {
    Capture,
    Pivot
  };

  State state_ = State::Capture;
  skills::Capture capture_;
  


  RobotCommand Capture(const World & world, const Robot & robot);
  RobotCommand Pivot(const World & world, const Robot & robot, const ateam_geometry::Point & block_point);
};

}  // namespace ateam_kenobi::skills

#endif  // SKILLS__PROTECT_HPP_
