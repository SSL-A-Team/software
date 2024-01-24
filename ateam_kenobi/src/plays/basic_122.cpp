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
#include "robot_assignment.hpp"

namespace ateam_kenobi::plays
{

Basic122::Basic122(visualization::PlayInfoPublisher & pip)
: BasePlay("Basic122", pip),
  striker_skill_(getOverlays().getChild("striker")),
  blockers_skill_(getOverlays().getChild("blockers")),
  goalie_skill_(getOverlays().getChild("goalie"), pip)
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

  assignAndRunStriker(available_robots, world, motion_commands);

  assignAndRunBlockers(available_robots, world, motion_commands);

  play_info_publisher_.send_play_message("Basic122");

  return motion_commands;
}

void Basic122::assignAndRunStriker(
  std::vector<Robot> & available_robots, const World & world,
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
  16> & motion_commands)
{
  std::vector<ateam_geometry::Point> assignable_positions;
  assignable_positions.push_back(striker_skill_.getAssignmentPoint(world));
  auto assignment_map = robot_assignment::assign(available_robots, assignable_positions);
  if (assignment_map.empty()) {
    return;
  }
  const auto robot_id = assignment_map.begin()->first;
  auto maybe_robot = world.our_robots[robot_id];
  if (!maybe_robot) {
    return;
  }
  striker_skill_.setTargetPoint(ateam_geometry::Point(world.field.field_length, 0.0));
  motion_commands[robot_id] = striker_skill_.runFrame(world, maybe_robot.value());

  play_helpers::removeRobotWithId(available_robots, robot_id);
}

void Basic122::assignAndRunBlockers(
  std::vector<Robot> & available_robots,
  const World & world,
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
  16> & motion_commands)
{
  std::vector<ateam_geometry::Point> assignable_positions;
  auto blockable_robot_assignment_goals = blockers_skill_.getAssignmentPoints(world);
  std::copy_n(
    blockable_robot_assignment_goals.begin(), std::min(
      2ul,
      blockable_robot_assignment_goals.size()),
    std::back_inserter(assignable_positions));
  auto assignment_map = robot_assignment::assign(available_robots, assignable_positions);
  std::vector<Robot> assigned_robots;
  for (auto [robot_id, pos_ind] : assignment_map) {
    auto maybe_robot = world.our_robots[robot_id];
    if (!maybe_robot) {
      continue;
    }
    const auto & robot = maybe_robot.value();
    assigned_robots.push_back(robot);
  }
  auto skill_commands = blockers_skill_.runFrame(world, assigned_robots);
  for (auto robot_ind = 0ul; robot_ind < assigned_robots.size(); ++robot_ind) {
    const auto robot_id = assigned_robots[robot_ind].id;
    motion_commands[robot_id] = skill_commands[robot_ind];
    play_helpers::removeRobotWithId(available_robots, robot_id);
  }
}
}  // namespace ateam_kenobi::plays
