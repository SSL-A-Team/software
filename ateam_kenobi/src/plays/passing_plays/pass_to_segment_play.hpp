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

#ifndef PLAYS__PASSING_PLAYS__PASS_TO_SEGMENT_PLAY_HPP_
#define PLAYS__PASSING_PLAYS__PASS_TO_SEGMENT_PLAY_HPP_

#include <ateam_geometry/types.hpp>
#include "stp/play.hpp"
#include "tactics/standard_defense.hpp"
#include "tactics/pass_to_segment.hpp"
#include "skills/lane_idler.hpp"

namespace ateam_kenobi::plays
{

/**
 * @brief Base play for all pass to segment plays
 *
 * This play class is not intended to be used directly as a play. Instead, real plays should
 * inherit from this play and call @c setTarget to control the pass target.
 *
 * This play assumes you are using lanes, so the third offense bot not involved in passing will
 * run the LaneIdler skill. The lane to idle in will be guessed based on the ball and target
 * locations.
 */
class PassToSegmentPlay : public stp::Play
{
public:
  using TargetSelectionFunc = std::function<ateam_geometry::Segment(const World & world)>;

  PassToSegmentPlay(stp::Options stp_options, TargetSelectionFunc target_func);

  virtual ~PassToSegmentPlay() = default;

  stp::PlayScore getScore(const World & world) override;

  void reset() override;

  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
    16> runFrame(const World & world) override;

private:
  TargetSelectionFunc target_func_;
  tactics::StandardDefense defense_tactic_;
  tactics::PassToSegment pass_tactic_;
  skills::LaneIdler idler_skill_;

  play_helpers::lanes::Lane getIdleLane(
    const World & world,
    const ateam_geometry::Segment & target);
};

}  // namespace ateam_kenobi::plays

#endif  // PLAYS__PASSING_PLAYS__PASS_TO_SEGMENT_PLAY_HPP_
