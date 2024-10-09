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

#include "their_kickoff_play.hpp"
#include <ranges>
#include <algorithm>
#include <limits>
#include <vector>
#include <ateam_geometry/intersection.hpp>
#include "play_helpers/robot_assignment.hpp"
#include "play_helpers/available_robots.hpp"

namespace ateam_kenobi::plays
{

TheirKickoffPlay::TheirKickoffPlay(stp::Options stp_options)
: stp::Play(kPlayName, stp_options),
  easy_move_tos_(createIndexedChildren<play_helpers::EasyMoveTo>("EasyMoveTo")),
  defense_(createChild<tactics::StandardDefense>("defense"))
{}

stp::PlayScore TheirKickoffPlay::getScore(const World & world)
{
  if (world.in_play) {
    return stp::PlayScore::NaN();
  }
  const auto & cmd = world.referee_info.running_command;
  const auto & prev = world.referee_info.prev_command;
  if (cmd == ateam_common::GameCommand::PrepareKickoffTheirs ||
    (cmd == ateam_common::GameCommand::NormalStart &&
    prev == ateam_common::GameCommand::PrepareKickoffTheirs))
  {
    return stp::PlayScore::Max();
  }
  return stp::PlayScore::NaN();
}

void TheirKickoffPlay::reset()
{
  defense_.reset();
  for (auto & emt : easy_move_tos_) {
    emt.reset();
  }
}

std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
  16> TheirKickoffPlay::runFrame(const World & world)
{
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> motion_commands;

  auto available_robots = play_helpers::getAvailableRobots(world);
  play_helpers::removeGoalie(available_robots, world);

  const auto offense_points = getOffensePoints(world);

  play_helpers::GroupAssignmentSet groups;
  groups.AddGroup("defense", defense_.getAssignmentPoints(world));
  groups.AddGroup("offense", offense_points);

  const auto assignments = play_helpers::assignGroups(available_robots, groups);

  defense_.runFrame(world, assignments.GetGroupFilledAssignments("defense"), motion_commands);

  runOffense(world, offense_points, assignments.GetGroupAssignments("offense"), motion_commands);

  const auto offense_assignments = assignments.GetGroupAssignments("offense");

  return motion_commands;
}

std::vector<ateam_geometry::Point> TheirKickoffPlay::getOffensePoints(const World & world)
{
  const double x = -1.0;
  std::vector<ateam_geometry::Point> points;

  points.push_back(ateam_geometry::Point(x, 0.0));

  auto opponents = play_helpers::getVisibleRobots(world.their_robots);

  auto neg_y = [](const Robot & bot) {return bot.pos.y() < -kRobotRadius;};
  auto pos_y = [](const Robot & bot) {return bot.pos.y() > kRobotRadius;};

  const auto neg_part_end = std::partition(opponents.begin(), opponents.end(), neg_y);

  const auto pos_part_end = std::partition(neg_part_end, opponents.end(), pos_y);

  /* All robots on the left side of the field are between opponents.begin() and neg_part_end.
   * All robots on the right side of the field are between neg_part_end and pos_part_end.
   * All robots on the center line are between pos_part_end and opponents.end().
   */

  auto by_x = [](const Robot & a, const Robot & b) {return a.pos.x() < b.pos.x();};

  const auto closest_neg_opponent_iter = std::min_element(opponents.begin(), neg_part_end, by_x);
  const auto closest_pos_opponent_iter = std::min_element(neg_part_end, pos_part_end, by_x);

  const ateam_geometry::Point goal_center{-world.field.field_length / 2.0, 0.0};

  const ateam_geometry::Point neg_fallback{x, -world.field.field_width / 4.0};
  if (closest_neg_opponent_iter != neg_part_end) {
    const auto opponent_pos = closest_neg_opponent_iter->pos;
    points.push_back(getOffensePointToBlockTarget(world, opponent_pos, x, neg_fallback));
    getOverlays().drawLine("neg_target_line", {goal_center, opponent_pos}, "blue");
  } else {
    points.push_back(neg_fallback);
  }

  const ateam_geometry::Point pos_fallback{x, world.field.field_width / 4.0};
  if (closest_pos_opponent_iter != pos_part_end) {
    const auto opponent_pos = closest_pos_opponent_iter->pos;
    points.push_back(getOffensePointToBlockTarget(world, opponent_pos, x, pos_fallback));
    getOverlays().drawLine("pos_target_line", {goal_center, opponent_pos}, "blue");
  } else {
    points.push_back(pos_fallback);
  }

  return points;
}

ateam_geometry::Point TheirKickoffPlay::getOffensePointToBlockTarget(
  const World & world,
  const ateam_geometry::Point & target, const double & x, const ateam_geometry::Point & fallback)
{
  const ateam_geometry::Point goal_center{-world.field.field_length / 2.0, 0.0};
  const ateam_geometry::Segment goal_target_segment{goal_center, target};
  const ateam_geometry::Segment x_segment{
    ateam_geometry::Point{x, -world.field.field_width / 2.0},
    ateam_geometry::Point{x, world.field.field_width / 2.0}
  };

  const auto maybe_intersection = ateam_geometry::intersection(goal_target_segment, x_segment);

  if (!maybe_intersection) {
    return fallback;
  }

  if (!std::holds_alternative<ateam_geometry::Point>(*maybe_intersection)) {
    return fallback;
  }

  return std::get<ateam_geometry::Point>(*maybe_intersection);
}

void TheirKickoffPlay::runOffense(
  const World & world,
  const std::vector<ateam_geometry::Point> & points,
  const std::vector<std::optional<Robot>> & robots,
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> & motion_commands)
{
  const auto num_runable = std::min(points.size(), robots.size());

  for (auto i = 0ul; i < num_runable; ++i) {
    const auto & maybe_robot = robots[i];
    if (!maybe_robot) {
      continue;
    }
    const auto & robot = *maybe_robot;
    const auto & point = points[i];
    auto & emt = easy_move_tos_[robot.id];
    emt.setTargetPosition(point);
    emt.face_absolute(0.0);
    motion_commands[robot.id] = emt.runFrame(robot, world);
  }
}

}  // namespace ateam_kenobi::plays
