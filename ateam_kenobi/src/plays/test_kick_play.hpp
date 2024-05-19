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


#ifndef PLAYS__TEST_KICK_PLAY_HPP_
#define PLAYS__TEST_KICK_PLAY_HPP_

#include "stp/play.hpp"
#include "skills/line_kick.hpp"
#include "play_helpers/available_robots.hpp"

namespace ateam_kenobi::plays
{

class TestKickPlay : public stp::Play
{
public:
  TestKickPlay()
  : stp::Play("TestKickPlay"),
    line_kick_skill_(createChild<skills::LineKick>("line_kick"))
  {}

  void reset() override {}

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
    line_kick_skill_.setTargetPoint(ateam_geometry::Point(world.field.field_length / 2.0, 0.0));
    motion_commands[robot.id] = line_kick_skill_.runFrame(world, robot);
    return motion_commands;
  }

private:
  skills::LineKick line_kick_skill_;
};

}  // namespace ateam_kenobi::plays

#endif  // PLAYS__TEST_KICK_PLAY_HPP_
