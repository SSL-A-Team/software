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
#include "play_helpers/available_robots.hpp"
#include "play_helpers/robot_assignment.hpp"

namespace ateam_kenobi::plays
{

KickOnGoalPlay::KickOnGoalPlay(stp::Options stp_options)
: stp::Play(kPlayName, stp_options),
  defense_(createChild<tactics::StandardDefense>("defense")),
  striker_(createChild<skills::LineKick>("striker"))
{
  setEnabled(false);  // TODO(barulicm) remove when ready
}

double KickOnGoalPlay::getScore(const World & world)
{
  /* TODO(barulicm) estimate likelihood of shot success and / or just do it based on how far down
   * the field we are
   */
  (void)world;
  return 0.0;
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

  const ateam_geometry::Point opp_goal_center{world.field.field_length/2.0, 0};
  striker_.SetTargetPoint(opp_goal_center);

  play_helpers::GroupAssignmentSet groups;
  groups.AddGroup("defense", defense_.getAssignmentPoints(world));
  groups.AddPosition("striker", striker_.GetAssignmentPoint(world));

  auto assignments = play_helpers::assignGroups(available_robots, groups);

  defense_.runFrame(world, assignments.GetGroupFilledAssignments("defense"), motion_commands);

  const auto maybe_striker_bot = assignments.GetPositionAssignment("striker");
  if (maybe_striker_bot) {
    const auto & striker_bot = *maybe_striker_bot;
    motion_commands[striker_bot.id] = striker_.RunFrame(world, striker_bot);
  }

  return motion_commands;
}

}  // namespace ateam_kenobi::plays
