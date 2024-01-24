// Copyright 2023 A Team
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

#ifndef PLAYS__OUR_KICKOFF_PLAY_HPP_
#define PLAYS__OUR_KICKOFF_PLAY_HPP_

#include <vector>

#include "base_play.hpp"
#include "path_planning/path_planner.hpp"
#include "motion/motion_controller.hpp"
#include "ateam_geometry/types.hpp"
#include "ateam_common/game_controller_listener.hpp"
#include "types/robot.hpp"
#include "skills/goalie.hpp"
#include "skills/line_kick.hpp"

namespace ateam_kenobi::plays
{
class OurKickoffPlay : public BasePlay
{
public:
  OurKickoffPlay();

  void reset() override;

  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
    16> runFrame(const World & world) override;

  void set_kickoff_ready();

private:
  skills::LineKick line_kick_skill_;
  skills::Goalie goalie_skill_;
  std::vector<ateam_geometry::Point> defender_positions_;

  std::array<play_helpers::EasyMoveTo, 16> easy_move_tos_;

  int prev_id_ = -1;
  const std::optional<Robot> maybe_kicker_;
  std::array<bool, 16> attempted_to_kick_;
};
}  // namespace ateam_kenobi::plays

#endif  // PLAYS__OUR_KICKOFF_PLAY_HPP_
