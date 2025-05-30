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

#include <vector>
#include "core/stp/play.hpp"
#include "tactics/standard_defense.hpp"
#include "tactics/pass.hpp"
#include "skills/lane_idler.hpp"

#ifndef PLAYS__PASSING_PLAYS__SPATIAL_PASS_PLAY_HPP_
#define PLAYS__PASSING_PLAYS__SPATIAL_PASS_PLAY_HPP_

namespace ateam_kenobi::plays
{

class SpatialPassPlay : public stp::Play
{
public:
  static constexpr const char * kPlayName = "SpatialPassPlay";

  explicit SpatialPassPlay(stp::Options stp_options);

  virtual ~SpatialPassPlay() = default;

  stp::PlayScore getScore(const World & world) override;

  stp::PlayCompletionState getCompletionState() override;

  void reset() override;

  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
    16> runFrame(const World & world) override;

private:
  tactics::StandardDefense defense_tactic_;
  tactics::Pass pass_tactic_;
  skills::LaneIdler idler_skill_;
  ateam_geometry::Point target_;
  bool started_ = false;
  std::vector<uint8_t> spatial_map_rendering_data_;

  play_helpers::lanes::Lane getIdleLane(const World & world);
};

}  // namespace ateam_kenobi::plays

#endif  // PLAYS__PASSING_PLAYS__SPATIAL_PASS_PLAY_HPP_
