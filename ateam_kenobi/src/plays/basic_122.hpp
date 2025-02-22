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


#ifndef PLAYS__BASIC_122_HPP_
#define PLAYS__BASIC_122_HPP_

#include <vector>
#include "stp/play.hpp"
#include "skills/line_kick.hpp"
#include "tactics/blockers.hpp"
#include "tactics/standard_defense.hpp"

namespace ateam_kenobi::plays
{

class Basic122 : public stp::Play
{
public:
  static constexpr const char * kPlayName = "Basic122";

  explicit Basic122(stp::Options stp_options);

  stp::PlayScore getScore(const World & world) override;

  void reset() override;

  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> runFrame(const World & world) override;

private:
  skills::LineKick striker_skill_;
  tactics::Blockers blockers_skill_;
  tactics::StandardDefense defense_;

  void runStriker(
    const Robot & striker_bot, const World & world,
    ateam_msgs::msg::RobotMotionCommand & motion_command);

  void runBlockers(
    const std::vector<Robot> & blocker_bots, const World & world,
    std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
    16> & motion_commands);

  bool doTheyHavePossession(const World & world);
};

}  // namespace ateam_kenobi::plays

#endif  // PLAYS__BASIC_122_HPP_
