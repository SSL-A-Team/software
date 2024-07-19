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

#include "kick_on_goal_play.hpp"
#include <limits>
#include "play_helpers/available_robots.hpp"
#include "play_helpers/robot_assignment.hpp"
#include "play_helpers/window_evaluation.hpp"
#include "play_helpers/shot_evaluation.hpp"

namespace ateam_kenobi::plays
{

KickOnGoalPlay::KickOnGoalPlay(stp::Options stp_options)
: stp::Play(kPlayName, stp_options),
  defense_(createChild<tactics::StandardDefense>("defense")),
  striker_(createChild<skills::LineKick>("striker")),
  lane_idler_a_(createChild<skills::LaneIdler>("lane_idler_a")),
  lane_idler_b_(createChild<skills::LaneIdler>("lane_idler_b"))
{
}

stp::PlayScore KickOnGoalPlay::getScore(const World & world)
{
  if (world.referee_info.running_command != ateam_common::GameCommand::NormalStart &&
    world.referee_info.running_command != ateam_common::GameCommand::ForceStart &&
    world.referee_info.running_command != ateam_common::GameCommand::DirectFreeOurs)
  {
    return stp::PlayScore::NegativeInfinity();
  }

  if (world.ball.pos.x() < 0.0) {
    return stp::PlayScore::Min();
  }

  return play_helpers::GetShotSuccessChance(world, world.ball.pos);
}

void KickOnGoalPlay::reset()
{
  defense_.reset();
  striker_.Reset();
}

std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> KickOnGoalPlay::runFrame(
  const World & world)
{
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> motion_commands;

  auto available_robots = play_helpers::getAvailableRobots(world);
  play_helpers::removeGoalie(available_robots, world);


  const ateam_geometry::Segment their_goal_segment{
    ateam_geometry::Point{world.field.field_length / 2.0, -world.field.goal_width / 2.0},
    ateam_geometry::Point{world.field.field_length / 2.0, world.field.goal_width / 2.0, }
  };
  const auto windows = play_helpers::window_evaluation::getWindows(
    their_goal_segment,
    world.ball.pos, play_helpers::getVisibleRobots(world.their_robots));
  play_helpers::window_evaluation::drawWindows(windows, world.ball.pos, getOverlays());
  const auto largest_window = play_helpers::window_evaluation::getLargestWindow(windows);
  if (largest_window) {
    striker_.SetTargetPoint(CGAL::midpoint(*largest_window));
  } else {
    const ateam_geometry::Point opp_goal_center{world.field.field_length / 2.0, 0};
    striker_.SetTargetPoint(opp_goal_center);
  }

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
  groups.AddPosition("striker", striker_.GetAssignmentPoint(world));
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

  if (enough_bots_for_defense) {
    defense_.runFrame(world, assignments.GetGroupFilledAssignments("defense"), motion_commands);
  }

  assignments.RunPositionIfAssigned(
    "striker", [this, &world, &motion_commands](const Robot & robot) {
      motion_commands[robot.id] = striker_.RunFrame(world, robot);
    });

  if (enough_bots_for_idlers) {
    assignments.RunPositionIfAssigned(
      "lane_idler_a", [this, &world, &motion_commands](const Robot & robot) {
        motion_commands[robot.id] = lane_idler_a_.RunFrame(world, robot);
      });

    assignments.RunPositionIfAssigned(
      "lane_idler_b", [this, &world, &motion_commands](const Robot & robot) {
        motion_commands[robot.id] = lane_idler_b_.RunFrame(world, robot);
      });
  }

  return motion_commands;
}

}  // namespace ateam_kenobi::plays
