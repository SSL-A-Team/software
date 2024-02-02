// Copyright 2021 A Team
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


#include "basic_122.hpp"
#include <vector>
#include "play_helpers/available_robots.hpp"
#include "play_helpers/robot_assignment.hpp"

namespace ateam_kenobi::plays
{

Basic122::Basic122()
: BasePlay("Basic122"),
  striker_skill_(getOverlays().getChild("striker")),
  blockers_skill_(getOverlays().getChild("blockers")),
  goalie_skill_(getOverlays().getChild("goalie"))
{
}

void Basic122::reset()
{
  blockers_skill_.reset();
  goalie_skill_.reset();
}

std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> Basic122::runFrame(
  const World & world)
{
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> motion_commands;

  auto available_robots = play_helpers::getAvailableRobots(world);
  play_helpers::removeGoalie(available_robots, world);

  goalie_skill_.runFrame(world, motion_commands);

  std::vector<ateam_geometry::Point> assignment_positions;
  assignment_positions.push_back(striker_skill_.getAssignmentPoint(world));
  const auto blocker_assignment_positions = blockers_skill_.getAssignmentPoints(world);
  assignment_positions.insert(
    assignment_positions.end(),
    blocker_assignment_positions.begin(), blocker_assignment_positions.end());

  std::vector<std::vector<int>> disallowed_ids;
  if (world.double_touch_forbidden_id_) {
    std::fill_n(std::back_inserter(disallowed_ids), assignment_positions.size(), std::vector<int>{});
    disallowed_ids[0].push_back(*world.double_touch_forbidden_id_);
  }

  const auto assignments = play_helpers::assignRobots(
    available_robots, assignment_positions,
    disallowed_ids);

  const auto & striker = assignments[0];

  if(striker) {
    runStriker(*striker, world, motion_commands[striker->id].emplace());
  }

  std::vector<Robot> blockers;
  for(auto ind = 1ul; ind < assignments.size(); ++ind) {
    if(assignments[ind]) {
      blockers.push_back(*assignments[ind]);
    }
  }

  runBlockers(blockers, world, motion_commands);

  return motion_commands;
}

void Basic122::runStriker(
  const Robot & striker_bot, const World & world,
  ateam_msgs::msg::RobotMotionCommand & motion_command)
{
  striker_skill_.setTargetPoint(ateam_geometry::Point(world.field.field_length / 2, 0.0));
  motion_command = striker_skill_.runFrame(world, striker_bot);
}

void Basic122::runBlockers(
  const std::vector<Robot> & blocker_bots,
  const World & world,
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
  16> & motion_commands)
{
  const auto skill_commands = blockers_skill_.runFrame(world, blocker_bots);

  for (auto robot_ind = 0ul; robot_ind < blocker_bots.size(); ++robot_ind) {
    motion_commands[blocker_bots[robot_ind].id] = skill_commands[robot_ind];
  }
}
}  // namespace ateam_kenobi::plays
