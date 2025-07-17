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

#include "defense_play.hpp"
#include <vector>
#include "core/play_helpers/possession.hpp"
#include "core/play_helpers/available_robots.hpp"
#include "core/play_helpers/robot_assignment.hpp"

namespace ateam_kenobi::plays
{

DefensePlay::DefensePlay(stp::Options stp_options)
: stp::Play(kPlayName, stp_options),
  defense_tactic_(createChild<tactics::StandardDefense>("defense")),
  blockers_(createChild<tactics::Blockers>("blockers"))
{
  // setEnabled(false);
  blockers_.setMaxBlockerCount(3);
}

stp::PlayScore DefensePlay::getScore(const World & world)
{
  if (!world.in_play &&
    world.referee_info.running_command != ateam_common::GameCommand::ForceStart &&
    world.referee_info.running_command != ateam_common::GameCommand::NormalStart &&
    world.referee_info.running_command != ateam_common::GameCommand::DirectFreeOurs)
  {
    return stp::PlayScore::NaN();
  }

  if (play_helpers::WhoHasPossession(world) == play_helpers::PossessionResult::Theirs) {
    return stp::PlayScore::Max();
  } else {
    return stp::PlayScore::Min();
  }
}

void DefensePlay::reset()
{
  defense_tactic_.reset();
  blockers_.reset();
}

std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> DefensePlay::runFrame(
  const World & world)
{
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> motion_commands;

  auto available_robots = play_helpers::getAvailableRobots(world);
  play_helpers::removeGoalie(available_robots, world);

  play_helpers::GroupAssignmentSet groups;
  groups.AddGroup("defense", defense_tactic_.getAssignmentPoints(world));
  groups.AddGroup("blockers", blockers_.getAssignmentPoints(world));
  const auto assignments = play_helpers::assignGroups(available_robots, groups);

  defense_tactic_.runFrame(
    world, assignments.GetGroupFilledAssignments("defense"),
    motion_commands);

  getPlayInfo()["defense"] = defense_tactic_.getPlayInfo();

  std::vector<Robot> blockers = assignments.GetGroupFilledAssignments("blockers");
  const auto blocker_commands =
    blockers_.runFrame(world, assignments.GetGroupFilledAssignments("blockers"));
  for (auto robot_ind = 0ul; robot_ind < blockers.size(); ++robot_ind) {
    motion_commands[blockers[robot_ind].id] = blocker_commands[robot_ind];
  }

  return motion_commands;
}

}  // namespace ateam_kenobi::plays
