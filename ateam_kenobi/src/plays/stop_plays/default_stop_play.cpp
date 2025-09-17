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

#include "default_stop_play.hpp"
#include <ranges>
#include <algorithm>
#include <limits>
#include <ateam_msgs/msg/robot_motion_command.hpp>
#include <ateam_common/robot_constants.hpp>
#include <ateam_geometry/normalize.hpp>
#include <ateam_geometry/do_intersect.hpp>
#include "core/play_helpers/window_evaluation.hpp"
#include "core/play_helpers/available_robots.hpp"
#include "core/play_helpers/robot_assignment.hpp"
#include "core/path_planning/obstacles.hpp"
#include "core/path_planning/escape_velocity.hpp"
#include "stop_helpers.hpp"

namespace helpers = ateam_kenobi::plays::stop_plays::stop_helpers;

namespace ateam_kenobi::plays
{
DefaultStopPlay::DefaultStopPlay(stp::Options stp_options)
: stp::Play(kPlayName, stp_options)
{
}

stp::PlayScore DefaultStopPlay::getScore(const World & world)
{
  switch (world.referee_info.running_command) {
    case ateam_common::GameCommand::Stop:
      return 50.0;
    default:
      return stp::PlayScore::Min();
  }
}

std::array<std::optional<RobotCommand>, 16> DefaultStopPlay::runFrame(
  const World & world)
{
  const auto added_obstacles = helpers::getAddedObstacles(world);

  helpers::drawObstacles(world, added_obstacles, getOverlays(), getLogger());

  std::array<std::optional<RobotCommand>, 16> motion_commands;

  helpers::moveBotsTooCloseToBall(world, added_obstacles, motion_commands, getOverlays(),
      getPlayInfo());

  helpers::moveBotsInObstacles(world, added_obstacles, motion_commands, getPlayInfo());

  // Halt all robots that weren't already assigned a motion command
  std::ranges::replace_if(
    motion_commands,
    [](const auto & o) {return !o;}, std::make_optional(RobotCommand{}));

  // Rules say <1.5m/s. We'll use 1m/s to give some room for error.
  for(auto & maybe_cmd : motion_commands) {
    if(!maybe_cmd) continue;
    maybe_cmd->motion_intent.motion_options.max_velocity = 1.0;
  }

  return motion_commands;
}

}  // namespace ateam_kenobi::plays
