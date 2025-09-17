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

#include "offensive_stop_play.hpp"
#include <ranges>
#include <algorithm>
#include <limits>
#include <vector>
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
OffensiveStopPlay::OffensiveStopPlay(stp::Options stp_options)
: stp::Play(kPlayName, stp_options)
{
}

stp::PlayScore OffensiveStopPlay::getScore(const World & world)
{
  if (world.referee_info.running_command != ateam_common::GameCommand::Stop) {
    return stp::PlayScore::Min();
  }
  if(!world.referee_info.next_command.has_value()) {
    return stp::PlayScore::Min();
  }
  switch (world.referee_info.next_command.value()) {
    case ateam_common::GameCommand::ForceStart:
    case ateam_common::GameCommand::PrepareKickoffOurs:
    case ateam_common::GameCommand::PreparePenaltyOurs:
    case ateam_common::GameCommand::DirectFreeOurs:
      return stp::PlayScore::Max();
    default:
      return stp::PlayScore::Min();
  }
}

std::array<std::optional<RobotCommand>, 16> OffensiveStopPlay::runFrame(
  const World & world)
{
  const auto added_obstacles = helpers::getAddedObstacles(world);

  helpers::drawObstacles(world, added_obstacles, getOverlays(), getLogger());

  std::array<std::optional<RobotCommand>, 16> motion_commands;

  runPrepBot(world, motion_commands);

  helpers::moveBotsTooCloseToBall(world, added_obstacles, motion_commands, getOverlays(),
      getPlayInfo());

  helpers::moveBotsInObstacles(world, added_obstacles, motion_commands, getPlayInfo());

  // Halt all robots that weren't already assigned a motion command
  std::ranges::replace_if(
    motion_commands,
    [](const auto & o) {return !o;}, std::make_optional(RobotCommand{}));

  // Rules say <1.5m/s. We'll use 1m/s to give some room for error.
  // TODO(barulicm): Set max velocity to 1.0

  return motion_commands;
}

void OffensiveStopPlay::runPrepBot(
  const World & world,
  std::array<std::optional<RobotCommand>, 16> & maybe_motion_commands)
{
  const auto their_goal_center = ateam_geometry::Point(world.field.field_length / 2.0, 0.0);
  // TODO(barulicm): May need to handle balls close to field edge smarter
  const auto target_position = world.ball.pos +
    (kPrepBotDistFromBall * ateam_geometry::normalize(world.ball.pos - their_goal_center));

  if (!path_planning::IsPointInBounds(target_position, world)) {
    return;
  }

  auto available_robots = play_helpers::getAvailableRobots(world);
  play_helpers::removeGoalie(available_robots, world);
  if(available_robots.empty()) {
    // No available robots, nothing to do
    return;
  }
  const auto closest_bot = play_helpers::getClosestRobot(available_robots, target_position);

  // Wait for the closest bot to be out of the ball keepout circle before giving it new commands
  if(maybe_motion_commands[closest_bot.id]) {
    return;
  }

  RobotCommand command;
  command.motion_intent.linear = motion::intents::linear::PositionIntent{target_position};
  command.motion_intent.angular = motion::intents::angular::FacingIntent{world.ball.pos};
  command.motion_intent.obstacles = {
    ateam_geometry::makeDisk(world.ball.pos, stop_plays::stop_helpers::kKeepoutRadiusRules)
  };
  maybe_motion_commands[closest_bot.id] = command;
}

}  // namespace ateam_kenobi::plays
