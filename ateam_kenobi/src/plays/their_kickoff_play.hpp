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

#ifndef PLAYS__THEIR_KICKOFF_PLAY_HPP_
#define PLAYS__THEIR_KICKOFF_PLAY_HPP_

#include <array>
#include <vector>
#include "stp/play.hpp"
#include "play_helpers/easy_move_to.hpp"
#include "tactics/standard_defense.hpp"

namespace ateam_kenobi::plays
{

class TheirKickoffPlay : public stp::Play
{
public:
  static constexpr const char * kPlayName = "TheirKickoffPlay";

  explicit TheirKickoffPlay(stp::Options stp_options);

  double getScore(const World & world) override;

  void reset() override;

  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
    16> runFrame(const World & world) override;

private:
  std::array<play_helpers::EasyMoveTo, 16> easy_move_tos_;
  tactics::StandardDefense defense_;

  std::vector<ateam_geometry::Point> getOffensePoints(const World & world);

  ateam_geometry::Point getOffensePointToBlockTarget(
    const World & world,
    const ateam_geometry::Point & target,
    const double & x,
    const ateam_geometry::Point & fallback);

  void runOffense(
    const World & world,
    const std::vector<ateam_geometry::Point> & points,
    const std::vector<std::optional<Robot>> & robots,
    std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> & motion_commands);
};

}  // namespace ateam_kenobi::plays

#endif  // PLAYS__THEIR_KICKOFF_PLAY_HPP_
