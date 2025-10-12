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

#include "defenders_only_play.hpp"
#include <vector>
#include "core/play_helpers/available_robots.hpp"
#include "core/play_helpers/robot_assignment.hpp"

namespace ateam_kenobi::plays
{

DefendersOnlyPlay::DefendersOnlyPlay(stp::Options stp_options)
: stp::Play(kPlayName, stp_options),
  defense_tactic_(createChild<tactics::StandardDefense>("defense"))
{
  setEnabled(false);
}

stp::PlayScore DefendersOnlyPlay::getScore(const World & world)
{
  if (world.referee_info.running_command == ateam_common::GameCommand::ForceStart ||
    world.referee_info.running_command == ateam_common::GameCommand::NormalStart ||
    world.referee_info.running_command == ateam_common::GameCommand::DirectFreeOurs ||
    world.referee_info.running_command == ateam_common::GameCommand::DirectFreeTheirs ||
    world.referee_info.running_command == ateam_common::GameCommand::PrepareKickoffOurs ||
    world.referee_info.running_command == ateam_common::GameCommand::PrepareKickoffTheirs)
  {
    return stp::PlayScore::Max();
  }
  return stp::PlayScore::NaN();
}

void DefendersOnlyPlay::reset()
{
  defense_tactic_.reset();
}

std::array<std::optional<RobotCommand>, 16> DefendersOnlyPlay::runFrame(
  const World & world)
{
  std::array<std::optional<RobotCommand>, 16> motion_commands;

  auto available_robots = play_helpers::getAvailableRobots(world);
  play_helpers::removeGoalie(available_robots, world);

  play_helpers::GroupAssignmentSet groups;
  groups.AddGroup("defense", defense_tactic_.getAssignmentPoints(world));
  const auto assignments = play_helpers::assignGroups(available_robots, groups);

  defense_tactic_.runFrame(
    world, assignments.GetGroupFilledAssignments("defense"),
    motion_commands);

  ForwardPlayInfo(defense_tactic_);

  return motion_commands;
}

}  // namespace ateam_kenobi::plays
