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
#include <ranges>
#include <algorithm>
#include <limits>
#include <vector>
#include <string>
#include "core/play_helpers/available_robots.hpp"
#include "core/play_helpers/robot_assignment.hpp"
#include "core/play_helpers/window_evaluation.hpp"

namespace ateam_kenobi::plays
{

Basic122::Basic122(stp::Options stp_options)
: stp::Play(kPlayName, stp_options),
  striker_skill_(createChild<skills::LineKick>("striker")),
  blockers_skill_(createChild<tactics::Blockers>("blockers")),
  defense_(createChild<tactics::StandardDefense>("defense"))
{
  setEnabled(false);
}

stp::PlayScore Basic122::getScore(const World & world)
{
  const auto default_score = 25.0;  // arbitrary score with room for other plays to be better
  switch (world.referee_info.running_command) {
    case ateam_common::GameCommand::ForceStart:
    case ateam_common::GameCommand::NormalStart:
    case ateam_common::GameCommand::DirectFreeOurs:
      return default_score;
    default:
      return world.in_play ? stp::PlayScore(default_score) : stp::PlayScore::Min();
  }
}

void Basic122::reset()
{
  blockers_skill_.reset();
  defense_.reset();
}

std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> Basic122::runFrame(
  const World & world)
{
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> motion_commands;

  auto available_robots = play_helpers::getAvailableRobots(world);
  play_helpers::removeGoalie(available_robots, world);

  play_helpers::GroupAssignmentSet groups;

  std::vector<int> disallowed_strikers;
  if (world.double_touch_forbidden_id_) {
    disallowed_strikers.push_back(*world.double_touch_forbidden_id_);
  }
  groups.AddPosition("striker", striker_skill_.GetAssignmentPoint(world), disallowed_strikers);

  groups.AddGroup("defenders", defense_.getAssignmentPoints(world));

  groups.AddGroup("blockers", blockers_skill_.getAssignmentPoints(world));

  const auto assignments = play_helpers::assignGroups(available_robots, groups);

  const auto & striker = assignments.GetPositionAssignment("striker");

  // using array to match formatting of other positions
  auto & striker_info = getPlayInfo()["positions"]["striker"] = nlohmann::json::array();
  if (striker) {
    runStriker(*striker, world, motion_commands[striker->id].emplace());
    striker_info.push_back(striker->id);
  }

  std::vector<Robot> defenders = assignments.GetGroupFilledAssignments("defenders");
  auto & defenders_info = getPlayInfo()["positions"]["defenders"] = nlohmann::json::array();
  std::ranges::transform(
    defenders, std::back_inserter(defenders_info), [](const Robot & r) {
      return r.id;
    });
  defense_.runFrame(world, defenders, motion_commands);

  std::vector<Robot> blockers = assignments.GetGroupFilledAssignments("blockers");
  auto & blockers_info = getPlayInfo()["positions"]["blockers"] = nlohmann::json::array();
  std::ranges::transform(
    blockers, std::back_inserter(blockers_info), [](const Robot & r) {
      return r.id;
    });
  runBlockers(blockers, world, motion_commands);

  return motion_commands;
}

void Basic122::runStriker(
  const Robot & striker_bot, const World & world,
  ateam_msgs::msg::RobotMotionCommand & motion_command)
{
  if (striker_skill_.IsDone()) {
    striker_skill_.Reset();
  }

  const auto they_have_possession = doTheyHavePossession(world);

  getPlayInfo()["Possession"] = they_have_possession ? "theirs" : "ours";

  if (they_have_possession) {
    const auto ball_to_bot_vec = striker_bot.pos - world.ball.pos;
    const auto vel = ateam_geometry::normalize(ball_to_bot_vec) * 0.25;
    if (ateam_geometry::norm(ball_to_bot_vec) < kRobotDiameter) {
      motion_command.twist.linear.x = vel.x();
      motion_command.twist.linear.y = vel.y();
    }
    return;
  }

  const ateam_geometry::Segment goal_segment(ateam_geometry::Point(
      world.field.field_length / 2,
      -world.field.goal_width / 2),
    ateam_geometry::Point(world.field.field_length / 2, world.field.goal_width / 2));
  auto robots = play_helpers::getVisibleRobots(world.our_robots);
  play_helpers::removeRobotWithId(robots, striker_bot.id);
  std::ranges::copy(play_helpers::getVisibleRobots(world.their_robots), std::back_inserter(robots));

  const auto windows = play_helpers::window_evaluation::getWindows(
    goal_segment, world.ball.pos,
    robots);
  play_helpers::window_evaluation::drawWindows(
    windows, world.ball.pos, getOverlays().getChild(
      "Windows"));
  const auto target_window = play_helpers::window_evaluation::getLargestWindow(windows);
  ateam_geometry::Point target_point;
  if (target_window) {
    target_point = CGAL::midpoint(*target_window);
  } else {
    target_point = ateam_geometry::Point(world.field.field_length / 2, 0.0);
  }
  striker_skill_.SetTargetPoint(target_point);
  striker_skill_.SetKickSpeed(3.0);
  motion_command = striker_skill_.RunFrame(world, striker_bot);
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

bool Basic122::doTheyHavePossession(const World & world)
{
  auto byDistToBall = [&world](const Robot & lhs, const Robot & rhs) {
      return CGAL::compare_distance_to_point(world.ball.pos, lhs.pos, rhs.pos) == CGAL::SMALLER;
    };

  const auto their_robots = play_helpers::getVisibleRobots(world.their_robots);
  if (their_robots.empty()) {
    return false;
  }

  const auto closest_their_robot = *std::ranges::min_element(their_robots, byDistToBall);

  if (CGAL::approximate_sqrt(
      CGAL::squared_distance(
        closest_their_robot.pos,
        world.ball.pos)) > kRobotRadius + 0.15)
  {
    return false;
  }

  const auto our_robots = play_helpers::getVisibleRobots(world.our_robots);

  if (our_robots.empty()) {
    return true;
  }

  const auto closest_our_robot = *std::ranges::min_element(our_robots, byDistToBall);

  return CGAL::compare_distance_to_point(
    world.ball.pos, closest_our_robot.pos,
    closest_their_robot.pos) == CGAL::LARGER;
}

}  // namespace ateam_kenobi::plays
