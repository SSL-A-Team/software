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

#include "test_play.hpp"
#include "ateam_geometry/types.hpp"
#include "core/types/world.hpp"
#include "skills/goalie.hpp"
#include "core/play_helpers/available_robots.hpp"

namespace ateam_kenobi::plays
{
TestPlay::TestPlay(stp::Options stp_options)
: stp::Play(kPlayName, stp_options),
  goalie_skill_(createChild<skills::Goalie>("goalie"))
{
}

void TestPlay::reset()
{
  goalie_skill_.reset();
}

std::array<std::optional<RobotCommand>, 16> TestPlay::runFrame(
  const World & world)
{
  std::array<std::optional<RobotCommand>, 16> maybe_motion_commands;
  auto current_available_robots = play_helpers::getAvailableRobots(world);
  play_helpers::removeGoalie(current_available_robots, world);

  if (current_available_robots.size() > 0) {
    const auto & robot = current_available_robots[0];
    int robot_id = robot.id;
    RobotCommand command;
    command.motion_intent.linear = motion::intents::linear::PositionIntent{world.ball.pos + ateam_geometry::Vector(-.2, 0)};
    command.motion_intent.angular = motion::intents::angular::FacingIntent{world.ball.pos};
    maybe_motion_commands.at(robot_id) = command;
  }

  goalie_skill_.runFrame(world, maybe_motion_commands);

  return maybe_motion_commands;
}
}  // namespace ateam_kenobi::plays
