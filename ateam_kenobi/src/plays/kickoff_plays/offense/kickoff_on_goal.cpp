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

#include "kickoff_on_goal.hpp"
#include "core/play_helpers/available_robots.hpp"
#include "core/play_helpers/robot_assignment.hpp"
#include "core/play_helpers/window_evaluation.hpp"

namespace ateam_kenobi::plays
{

KickoffOnGoalPlay::KickoffOnGoalPlay(stp::Options stp_options)
: stp::Play(kPlayName, stp_options),
  defense_(createChild<tactics::StandardDefense>("defense")),
  kick_(createChild<skills::UniversalKick>("kick")),
  multi_move_to_(createChild<tactics::MultiMoveTo>("support"))
{
  kick_.SetPreferredKickType(skills::UniversalKick::KickType::Line);
}

stp::PlayScore KickoffOnGoalPlay::getScore(const World & world)
{
  if (world.in_play) {
    return stp::PlayScore::NaN();
  }
  const auto & cmd = world.referee_info.running_command;
  const auto & prev = world.referee_info.prev_command;
  if (cmd == ateam_common::GameCommand::NormalStart &&
    prev == ateam_common::GameCommand::PrepareKickoffOurs)
  {
    const auto largest_window = getLargestWindowOnGoal(world);
    if (!largest_window) {
      return stp::PlayScore::Min();
    }
    return stp::PlayScore::Max() *
           (CGAL::approximate_sqrt(largest_window->squared_length()) / world.field.goal_width);
  }
  return stp::PlayScore::NaN();
}

void KickoffOnGoalPlay::reset()
{
  defense_.reset();
  kick_.Reset();
  multi_move_to_.Reset();
}

std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> KickoffOnGoalPlay::runFrame(
  const World & world)
{
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> motion_commands;

  auto available_robots = play_helpers::getAvailableRobots(world);
  play_helpers::removeGoalie(available_robots, world);

  const auto largest_window = getLargestWindowOnGoal(world);
  if (largest_window) {
    kick_.SetTargetPoint(CGAL::midpoint(*largest_window));
  } else {
    kick_.SetTargetPoint(ateam_geometry::Point{world.field.field_length / 2.0, 0.0});
  }

  multi_move_to_.SetTargetPoints(
    {
      ateam_geometry::Point(-0.3, world.field.field_width / 3),
      ateam_geometry::Point(-0.3, -world.field.field_width / 3)
    });
  multi_move_to_.SetFacePoint(world.ball.pos);

  play_helpers::GroupAssignmentSet groups;
  groups.AddPosition("kicker", kick_.GetAssignmentPoint(world));

  // const auto enough_bots_for_defense = available_robots.size() >= 3;
  // if (enough_bots_for_defense) {
  //   groups.AddGroup("defense", defense_.getAssignmentPoints(world));
  // }

  const auto enough_bots_for_supports = available_robots.size() >= 5;
  if (enough_bots_for_supports) {
    groups.AddGroup("supports", multi_move_to_.GetAssignmentPoints());
  }

  const auto assignments = play_helpers::assignGroups(available_robots, groups);

  assignments.RunPositionIfAssigned(
    "kicker", [this, &motion_commands, &world](const Robot & robot) {
      motion_commands[robot.id] = kick_.RunFrame(world, robot);
    });

  // defense_.runFrame(world, assignments.GetGroupFilledAssignmentsOrEmpty("defense"),
  //     motion_commands);

  if (enough_bots_for_supports) {
    multi_move_to_.RunFrame(world, assignments.GetGroupAssignments("supports"), motion_commands);
  }

  return motion_commands;
}

std::optional<ateam_geometry::Segment> KickoffOnGoalPlay::getLargestWindowOnGoal(
  const World & world)
{
  const ateam_geometry::Segment their_goal_segment{
    ateam_geometry::Point{world.field.field_length / 2.0, -world.field.goal_width / 2.0},
    ateam_geometry::Point{world.field.field_length / 2.0, world.field.goal_width / 2.0}
  };
  const auto windows = play_helpers::window_evaluation::getWindows(
    their_goal_segment,
    world.ball.pos, play_helpers::getVisibleRobots(
      world.their_robots));
  return play_helpers::window_evaluation::getLargestWindow(windows);
}
}  // namespace ateam_kenobi::plays
