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

#include "our_kickoff_prep_play.hpp"
#include <vector>
#include "core/play_helpers/available_robots.hpp"
#include "core/play_helpers/robot_assignment.hpp"
#include "plays/stop_plays/stop_helpers.hpp"

namespace helpers = ateam_kenobi::plays::stop_plays::stop_helpers;

namespace ateam_kenobi::plays
{

OurKickoffPrepPlay::OurKickoffPrepPlay(stp::Options stp_options)
: stp::Play(kPlayName, stp_options),
  defense_(createChild<tactics::StandardDefense>("defense")),
  multi_move_to_(createChild<tactics::MultiMoveTo>("multi_move_to"))
{}

stp::PlayScore OurKickoffPrepPlay::getScore(const World & world)
{
  const auto should_prep_during_stop = world.referee_info.next_command.has_value() &&
    world.referee_info.next_command.value() == ateam_common::GameCommand::PrepareKickoffOurs &&
    world.referee_info.running_command == ateam_common::GameCommand::Stop;

  if (world.referee_info.running_command == ateam_common::GameCommand::PrepareKickoffOurs) {
    return stp::PlayScore::Max();
  } else if (should_prep_during_stop) {
    return stp::PlayScore::Max();
  } else {
    return stp::PlayScore::NaN();
  }
}

void OurKickoffPrepPlay::reset()
{
  defense_.reset();
}

std::array<std::optional<RobotCommand>, 16> OurKickoffPrepPlay::runFrame(
  const World & world)
{
  std::array<std::optional<RobotCommand>, 16> motion_commands;

  auto available_robots = play_helpers::getAvailableRobots(world);
  play_helpers::removeGoalie(available_robots, world);

  std::vector<ateam_geometry::Point> move_to_targets = {
    ateam_geometry::Point{-0.25, 0}
  };

  // Offset the middle point backwards to avoid colliding with the ball during stop
  if (world.referee_info.running_command == ateam_common::GameCommand::Stop) {
    move_to_targets[0] = ateam_geometry::Point{-helpers::kKeepoutRadius, 0};
  }

  // TODO(barulicm): revert hacky fix for 2025 quals / replace with smarter fix (see git logs)
  if (available_robots.size() >= 2) {
    move_to_targets.push_back(ateam_geometry::Point(-0.3, world.field.field_width / 3));
  }
  if (available_robots.size() >= 3) {
    move_to_targets.push_back(ateam_geometry::Point(-0.3, -world.field.field_width / 3));
  }

  multi_move_to_.SetTargetPoints(move_to_targets);
  multi_move_to_.SetFaceAbsolue(0.0);

  play_helpers::GroupAssignmentSet groups;
  groups.AddGroup("movers", multi_move_to_.GetAssignmentPoints());
  const auto enough_bots_for_defense = available_robots.size() >= 4;
  if (enough_bots_for_defense) {
    groups.AddGroup("defense", defense_.getAssignmentPoints(world));
  }
  const auto assignments = play_helpers::assignGroups(available_robots, groups);

  defense_.runFrame(world, assignments.GetGroupFilledAssignmentsOrEmpty("defense"),
      motion_commands);
  multi_move_to_.RunFrame(assignments.GetGroupAssignments("movers"), motion_commands);

  if (world.referee_info.running_command == ateam_common::GameCommand::Stop) {
    // TODO(barulicm): find out what problem this was trying to fix

    // This is super hacky. Need to allow the center robot to move if
    // it is stuck up against the ball safety radius. No robots should be
    // trying to go near the ball when it is in the center so we should avoid it
    const bool need_to_avoid_ball =
      ateam_geometry::norm(ateam_geometry::Point{0, 0} - world.ball.pos) > 0.1;

    const auto added_obstacles = helpers::getAddedObstacles(world);
    helpers::drawObstacles(world, added_obstacles, getOverlays(), getLogger());
    helpers::moveBotsTooCloseToBall(world, added_obstacles, motion_commands, getOverlays(),
        getPlayInfo(), need_to_avoid_ball);

    helpers::moveBotsInObstacles(world, added_obstacles, motion_commands, getPlayInfo(), true);

    for (auto & motion_command : motion_commands) {
      if (motion_command.has_value()) {
        if(auto intent =
          std::get_if<motion::intents::Position>(&motion_command.value().motion_intent);
          intent != nullptr)
        {
          intent->limits.linear_velocity = 1.0;
        }
        if(auto intent =
          std::get_if<motion::intents::PositionFacing>(&motion_command.value().motion_intent);
          intent != nullptr)
        {
          intent->limits.linear_velocity = 1.0;
        }
      }
    }
  }

  return motion_commands;
}

}  // namespace ateam_kenobi::plays
