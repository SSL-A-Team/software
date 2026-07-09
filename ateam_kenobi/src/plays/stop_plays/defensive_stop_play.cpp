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
#include <vector>
#include <ateam_msgs/msg/robot_motion_command.hpp>
#include <ateam_common/robot_constants.hpp>
#include <ateam_geometry/normalize.hpp>
#include <ateam_geometry/do_intersect.hpp>
#include "core/play_helpers/window_evaluation.hpp"
#include "core/play_helpers/available_robots.hpp"
#include "core/play_helpers/robot_assignment.hpp"
#include "core/motion/path_planning/obstacles.hpp"
#include "core/motion/escape_velocity.hpp"
#include "stop_helpers.hpp"

namespace helpers = ateam_kenobi::plays::stop_plays::stop_helpers;

namespace ateam_kenobi::plays
{
DefensiveStopPlay::DefensiveStopPlay(stp::Options stp_options)
: stp::Play(kPlayName, stp_options),
  defense_tactic_(createChild<tactics::StandardDefense>("defense"))
{
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
    case ateam_common::GameCommand::PreparePenaltyTheirs:
    case ateam_common::GameCommand::DirectFreeTheirs:
      return stp::PlayScore::Max();
    case ateam_common::GameCommand::PrepareKickoffTheirs:
      return 50.0;  // Give kickoff prep a chance to run
    default:
      return stp::PlayScore::Min();
  }
}

std::array<std::optional<RobotCommand>, 16> DefensiveStopPlay::runFrame(
  const World & world)
{
  std::array<std::optional<RobotCommand>, 16> motion_commands;

  auto available_robots = play_helpers::getAvailableRobots(world);
  play_helpers::removeGoalie(available_robots, world);

  play_helpers::GroupAssignmentSet groups;
  std::vector<int> disallowed_strikers;
  if (world.double_touch_forbidden_id_) {
    disallowed_strikers.push_back(*world.double_touch_forbidden_id_);
  }
  groups.AddGroup("defense", defense_tactic_.getAssignmentPoints(world));
  const auto assignments = play_helpers::assignGroups(available_robots, groups);

  defense_tactic_.runFrame(world, assignments.GetGroupFilledAssignmentsOrEmpty("defense"),
      motion_commands);

  const auto added_obstacles = helpers::getAddedObstacles(world);

  helpers::drawObstacles(world, added_obstacles, getOverlays(), getLogger());

  runPrepBot(world, motion_commands);

  helpers::moveBotsTooCloseToBall(world, added_obstacles, motion_commands, getOverlays(),
      getPlayInfo(), true);

  helpers::moveBotsInObstacles(world, added_obstacles, motion_commands, getPlayInfo());

  // Halt all robots that weren't already assigned a motion command
  std::ranges::replace_if(
    motion_commands,
    [](const auto & o) {return !o;}, std::make_optional(RobotCommand{}));

  // Rules say <1.5m/s. We'll use 1m/s to give some room for error.
  for(auto & maybe_cmd : motion_commands) {
    if(!maybe_cmd) {continue;}
    std::visit([](auto & intent){
        using IntentType = std::decay_t<decltype(intent)>;
        if constexpr (motion::has_limits<IntentType>) {
          intent.limits.linear_velocity = 1.0;
        }
    }, maybe_cmd->motion_intent);
  }

  return motion_commands;
}

void DefensiveStopPlay::runPrepBot(
  const World & world,
  std::array<std::optional<RobotCommand>, 16> & maybe_motion_commands)
{
  const auto our_goal_center = ateam_geometry::Point(-world.field.field_length / 2.0, 0.0);
  const auto target_position = world.ball.pos +
    (kPrepBotDistFromBall * ateam_geometry::normalize(our_goal_center - world.ball.pos));

  if (!motion::path_planning::IsPointInBounds(target_position, world)) {
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

  motion::intents::PositionFacing intent;
  intent.position = target_position;
  intent.face_target = world.ball.pos;
  intent.obstacles = {
    ateam_geometry::makeDisk(world.ball.pos, stop_plays::stop_helpers::kKeepoutRadiusRules)
  };
  RobotCommand command;
  command.motion_intent = intent;
  maybe_motion_commands[closest_bot.id] = command;
}

}  // namespace ateam_kenobi::plays
