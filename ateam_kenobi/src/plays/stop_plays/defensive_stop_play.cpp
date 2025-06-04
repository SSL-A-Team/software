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

#include "defensive_stop_play.hpp"
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
DefensiveStopPlay::DefensiveStopPlay(stp::Options stp_options)
: stp::Play(kPlayName, stp_options)
{
  createIndexedChildren<play_helpers::EasyMoveTo>(easy_move_tos_, "EasyMoveTo");
  for (auto & move_to : easy_move_tos_) {
    // Rules say <1.5m/s. We'll use 1m/s to give some room for error.
    move_to.setMaxVelocity(1.0);
  }
  DefensiveStopPlay::reset();
}

stp::PlayScore DefensiveStopPlay::getScore(const World & world)
{
  if (world.referee_info.running_command != ateam_common::GameCommand::Stop) {
    return stp::PlayScore::Min();
  }
  if(!world.referee_info.next_command.has_value()) {
    return stp::PlayScore::Min();
  }
  switch (world.referee_info.next_command.value()) {
    case ateam_common::GameCommand::PrepareKickoffTheirs:
    case ateam_common::GameCommand::PreparePenaltyTheirs:
    case ateam_common::GameCommand::DirectFreeTheirs:
      return stp::PlayScore::Max();
    default:
      return stp::PlayScore::Min();
  }
}

void DefensiveStopPlay::reset()
{
  for (auto & move_to : easy_move_tos_) {
    move_to.reset();
  }
}

std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> DefensiveStopPlay::runFrame(
  const World & world)
{
  const auto added_obstacles = helpers::getAddedObstacles(world);

  helpers::drawObstacles(world, added_obstacles, getOverlays(), getLogger());

  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> motion_commands;

  helpers::moveBotsTooCloseToBall(world, added_obstacles, motion_commands, easy_move_tos_,
    getOverlays(), getPlayInfo());

  helpers::moveBotsInObstacles(world, added_obstacles, motion_commands, getPlayInfo());

  runPrepBot(world, motion_commands);

  // Halt all robots that weren't already assigned a motion command
  std::ranges::replace_if(
    motion_commands,
    [](const auto & o) {return !o;}, std::make_optional(ateam_msgs::msg::RobotMotionCommand{}));

  return motion_commands;
}

void DefensiveStopPlay::runPrepBot(
  const World & world,
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> & maybe_motion_commands)
{
  const auto our_goal_center = ateam_geometry::Point(-world.field.field_length / 2.0, 0.0);
  const auto target_position = world.ball.pos +
    (kPrepBotDistFromBall * (our_goal_center - world.ball.pos));

  auto available_robots = play_helpers::getAvailableRobots(world);
  play_helpers::removeGoalie(available_robots, world);
  const auto closest_bot = play_helpers::getClosestRobot(available_robots, target_position);

  auto & emt = easy_move_tos_[closest_bot.id];

  emt.setTargetPosition(target_position);
  emt.face_point(world.ball.pos);
  maybe_motion_commands[closest_bot.id] = emt.runFrame(closest_bot, world);
}

}  // namespace ateam_kenobi::plays
