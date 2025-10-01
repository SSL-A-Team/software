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

#ifndef PLAYS__KICK_ON_GOAL_PLAY_HPP_
#define PLAYS__KICK_ON_GOAL_PLAY_HPP_

#include "core/stp/play.hpp"
#include "tactics/standard_defense.hpp"
#include "skills/universal_kick.hpp"
#include "skills/lane_idler.hpp"

namespace ateam_kenobi::plays
{

class KickOnGoalPlay : public stp::Play
{
public:
  static constexpr const char * kPlayName = "KickOnGoalPlay";

  explicit KickOnGoalPlay(stp::Options stp_options);

  stp::PlayScore getScore(const World & world) override;

  void reset() override;

  std::array<std::optional<RobotCommand>, 16> runFrame(
    const World & world) override;

private:
  tactics::StandardDefense defense_;
  skills::UniversalKick striker_;
  skills::LaneIdler lane_idler_a_;
  skills::LaneIdler lane_idler_b_;
};

}  // namespace ateam_kenobi::plays

#endif  // PLAYS__KICK_ON_GOAL_PLAY_HPP_
