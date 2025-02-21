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


#ifndef PLAYS__TEST_PLAYS__TEST_KICK_PLAY_HPP_
#define PLAYS__TEST_PLAYS__TEST_KICK_PLAY_HPP_

#include "stp/play.hpp"
#include "skills/pivot_kick.hpp"
#include "skills/line_kick.hpp"
#include "play_helpers/available_robots.hpp"

namespace ateam_kenobi::plays
{

class TestKickPlay : public stp::Play
{
public:
  static constexpr const char * kPlayName = "TestKickPlay";

  explicit TestKickPlay(stp::Options stp_options)
  : stp::Play(kPlayName, stp_options),
    pivot_kick_skill_(createChild<skills::PivotKick>("pivot_kick")),
    line_kick_skill_(createChild<skills::LineKick>("line_kick"))
  {
    getParamInterface().declareParameter(kUsePivotKickParam, true);
  }

  void reset() override
  {
    line_kick_skill_.Reset();
    pivot_kick_skill_.Reset();
  }

  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
    16> runFrame(const World & world) override
  {
    std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> motion_commands;
    const auto & robots = play_helpers::getAvailableRobots(world);
    if (robots.empty()) {
      return {};
    }
    const auto robot = robots.front();

    // aim for center of opponent goal
    const ateam_geometry::Point target(-world.field.field_length / 2.0, 0.0);

    line_kick_skill_.SetTargetPoint(target);
    pivot_kick_skill_.SetTargetPoint(target);

    const auto use_pivot_kick = getParamInterface().getParameter<bool>(kUsePivotKickParam);

    motion_commands[robot.id] =
      use_pivot_kick ? pivot_kick_skill_.RunFrame(world, robot) : line_kick_skill_.RunFrame(
      world,
      robot);

    getPlayInfo()["line kick"] = line_kick_skill_.getPlayInfo();
    return motion_commands;
  }

private:
  char const * const kUsePivotKickParam = "use_pivot_kick";
  skills::PivotKick pivot_kick_skill_;
  skills::LineKick line_kick_skill_;
};

}  // namespace ateam_kenobi::plays

#endif  // PLAYS__TEST_PLAYS__TEST_KICK_PLAY_HPP_
