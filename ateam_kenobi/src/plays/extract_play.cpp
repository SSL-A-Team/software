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

#include "extract_play.hpp"
#include <limits>
#include "core/play_helpers/available_robots.hpp"
#include "core/play_helpers/robot_assignment.hpp"
#include "core/play_helpers/possession.hpp"

namespace ateam_kenobi::plays
{

ExtractPlay::ExtractPlay(stp::Options stp_options)
: stp::Play(kPlayName, stp_options),
  defense_(createChild<tactics::StandardDefense>("defense")),
  extract_(createChild<skills::Extract>("extract")),
  lane_idler_a_(createChild<skills::LaneIdler>("lane_idler_a")),
  lane_idler_b_(createChild<skills::LaneIdler>("lane_idler_b"))
{
  setEnabled(false);
}

stp::PlayScore ExtractPlay::getScore(const World & world)
{
  if (world.referee_info.running_command != ateam_common::GameCommand::NormalStart &&
    world.referee_info.running_command != ateam_common::GameCommand::ForceStart &&
    world.referee_info.running_command != ateam_common::GameCommand::DirectFreeOurs)
  {
    return stp::PlayScore::NegativeInfinity();
  }

  if (play_helpers::WhoHasPossession(world) == play_helpers::PossessionResult::Theirs) {
    return stp::PlayScore(100);  // arbitrary, high score to encourage being more aggressive
  }

  return stp::PlayScore::NaN();
}

void ExtractPlay::reset()
{
  defense_.reset();
  extract_.Reset();
}

std::array<std::optional<RobotCommand>, 16> ExtractPlay::runFrame(
  const World & world)
{
  std::array<std::optional<RobotCommand>, 16> motion_commands;

  auto available_robots = play_helpers::getAvailableRobots(world);
  play_helpers::removeGoalie(available_robots, world);

  if (play_helpers::lanes::IsBallInLane(world, play_helpers::lanes::Lane::LeftOffense)) {
    lane_idler_a_.SetLane(play_helpers::lanes::Lane::CenterOffense);
    lane_idler_b_.SetLane(play_helpers::lanes::Lane::RightOffense);
  } else if (play_helpers::lanes::IsBallInLane(world, play_helpers::lanes::Lane::CenterOffense)) {
    lane_idler_a_.SetLane(play_helpers::lanes::Lane::LeftOffense);
    lane_idler_b_.SetLane(play_helpers::lanes::Lane::RightOffense);
  } else if (play_helpers::lanes::IsBallInLane(world, play_helpers::lanes::Lane::RightOffense)) {
    lane_idler_a_.SetLane(play_helpers::lanes::Lane::LeftOffense);
    lane_idler_b_.SetLane(play_helpers::lanes::Lane::CenterOffense);
  }

  play_helpers::GroupAssignmentSet groups;
  groups.AddPosition("extract", extract_.GetAssignmentPoint(world));
  const auto enough_bots_for_defense = available_robots.size() >= 3;
  if (enough_bots_for_defense) {
    groups.AddGroup("defense", defense_.getAssignmentPoints(world));
  }
  const auto enough_bots_for_idlers = available_robots.size() >= 5;
  if (enough_bots_for_idlers) {
    groups.AddPosition("lane_idler_a", lane_idler_a_.GetAssignmentPoint(world));
    groups.AddPosition("lane_idler_b", lane_idler_b_.GetAssignmentPoint(world));
  }

  auto assignments = play_helpers::assignGroups(available_robots, groups);

  defense_.runFrame(world, assignments.GetGroupFilledAssignmentsOrEmpty("defense"),
      motion_commands);

  assignments.RunPositionIfAssigned(
    "extract", [this, &world, &motion_commands](const Robot & robot) {
      motion_commands[robot.id] = extract_.RunFrame(world, robot);
    });

  if (enough_bots_for_idlers) {
    assignments.RunPositionIfAssigned(
      "lane_idler_a", [this, &world, &motion_commands](const Robot & robot) {
        motion_commands[robot.id] = lane_idler_a_.RunFrame(world);
      });

    assignments.RunPositionIfAssigned(
      "lane_idler_b", [this, &world, &motion_commands](const Robot & robot) {
        motion_commands[robot.id] = lane_idler_b_.RunFrame(world);
      });
  }

  return motion_commands;
}

}  // namespace ateam_kenobi::plays
