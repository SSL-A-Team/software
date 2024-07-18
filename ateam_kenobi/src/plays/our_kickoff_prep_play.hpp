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

#ifndef PLAYS__OUR_KICKOFF_PREP_PLAY_HPP_
#define PLAYS__OUR_KICKOFF_PREP_PLAY_HPP_

#include "stp/play.hpp"
#include "tactics/standard_defense.hpp"
#include "tactics/multi_move_to.hpp"

namespace ateam_kenobi::plays
{

class OurKickoffPrepPlay : public stp::Play
{
public:
  static constexpr const char * kPlayName = "OurPenaltyPlay";

  explicit OurKickoffPrepPlay(stp::Options stp_options);

  stp::PlayScore getScore(const World & world) override;

  void reset() override;

  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> runFrame(
    const World & world);

private:
  tactics::StandardDefense defense_;
  tactics::MultiMoveTo multi_move_to_;

};

}  // namespace ateam_kenobi::plays

#endif  // PLAYS__OUR_KICKOFF_PREP_PLAY_HPP_
