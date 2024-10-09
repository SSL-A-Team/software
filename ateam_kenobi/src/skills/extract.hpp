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

#ifndef SKILLS__EXTRACT_HPP_
#define SKILLS__EXTRACT_HPP_

#include "stp/skill.hpp"
#include "play_helpers/easy_move_to.hpp"

namespace ateam_kenobi::skills
{

class Extract : public stp::Skill
{
public:
  explicit Extract(stp::Options stp_options);

  void Reset();

  ateam_msgs::msg::RobotMotionCommand RunFrame(const World & world, const Robot & robot);

  ateam_geometry::Point GetAssignmentPoint(const World & world)
  {
    return world.ball.pos;
  }

private:
  const int kBallsenseCountThreshold_ = 10;
  play_helpers::EasyMoveTo easy_move_to_;
  int ballsense_count_ = 0;
  std::optional<std::chrono::steady_clock::time_point> rip_start_time_;
};

}  // namespace ateam_kenobi::skills


#endif  // SKILLS__EXTRACT_HPP_
