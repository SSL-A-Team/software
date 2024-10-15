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

#include "pass_to_segment_play.hpp"
#include <algorithm>
#include <vector>
#include "play_helpers/window_evaluation.hpp"
#include "play_helpers/available_robots.hpp"
#include "play_helpers/robot_assignment.hpp"
#include "play_helpers/shot_evaluation.hpp"

namespace ateam_kenobi::plays
{

PassToSegmentPlay::PassToSegmentPlay(
  stp::Options stp_options,
  PassToSegmentPlay::TargetSelectionFunc target_func)
: stp::Play(stp_options.name, stp_options),
  target_func_(target_func),
  defense_tactic_(createChild<tactics::StandardDefense>("defense")),
  pass_tactic_(createChild<tactics::PassToSegment>("pass")),
  idler_skill_(createChild<skills::LaneIdler>("idler"))
{
}

stp::PlayScore PassToSegmentPlay::getScore(const World & world)
{
  // TODO(barulicm) does not work generically if we want to use this to get the ball into play
  if (!world.in_play &&
    world.referee_info.running_command != ateam_common::GameCommand::ForceStart &&
    world.referee_info.running_command != ateam_common::GameCommand::NormalStart &&
    world.referee_info.running_command != ateam_common::GameCommand::DirectFreeOurs)
  {
    return stp::PlayScore::NaN();
  }

  const auto target = target_func_(world);
  const auto visible_opponents = play_helpers::getVisibleRobots(world.their_robots);
  const auto windows = play_helpers::window_evaluation::getWindows(
    target, world.ball.pos,
    visible_opponents);
  const auto largest_window = play_helpers::window_evaluation::getLargestWindow(windows);
  if (!largest_window) {
    return stp::PlayScore::Min();
  }
  const auto target_point = CGAL::midpoint(*largest_window);

  const auto goal_chance = play_helpers::GetShotSuccessChance(world, target_point);

  // Even if there is no window now, things might change, so assume a non-zero success chance
  const double goal_chance_multiplier = std::max(goal_chance / 100.0, 0.2);

  // Arbitrary value that would give us plenty of room to catch a pass
  const auto ideal_target_length = kRobotDiameter * 5;
  const auto pass_chance_multiplier =
    std::clamp(
    largest_window->squared_length() / (ideal_target_length * ideal_target_length), 0.0, 1.0);
  return pass_chance_multiplier * goal_chance_multiplier * stp::PlayScore::Max();
}

stp::PlayCompletionState PassToSegmentPlay::getCompletionState()
{
  if (!started_) {
    return stp::PlayCompletionState::NotApplicable;
  }
  if (!pass_tactic_.isDone()) {
    return stp::PlayCompletionState::Busy;
  }
  return stp::PlayCompletionState::Done;
}

void PassToSegmentPlay::reset()
{
  defense_tactic_.reset();
  pass_tactic_.reset();
  idler_skill_.Reset();
  started_ = false;
  cached_target_ = ateam_geometry::Segment{};
}

std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> PassToSegmentPlay::runFrame(
  const World & world)
{
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> motion_commands;

  ateam_geometry::Segment target = cached_target_;

  if (!started_) {
    target = target_func_(world);
    cached_target_ = target;
  }

  getOverlays().drawLine("target", {target.source(), target.target()}, "grey");
  getOverlays().drawRectangle(
    "left_lane",
    play_helpers::lanes::GetLaneBounds(
      world,
      play_helpers::lanes::Lane::Left), "aqua", "transparent");
  getOverlays().drawRectangle(
    "center_lane",
    play_helpers::lanes::GetLaneBounds(
      world,
      play_helpers::lanes::Lane::Center), "aqua", "transparent");
  getOverlays().drawRectangle(
    "right_lane",
    play_helpers::lanes::GetLaneBounds(
      world,
      play_helpers::lanes::Lane::Right), "aqua", "transparent");

  pass_tactic_.setTarget(target);

  idler_skill_.SetLane(getIdleLane(world, target));

  auto available_robots = play_helpers::getAvailableRobots(world);
  play_helpers::removeGoalie(available_robots, world);

  play_helpers::GroupAssignmentSet groups;

  std::vector<int> disallowed_strikers;
  if (world.double_touch_forbidden_id_) {
    disallowed_strikers.push_back(*world.double_touch_forbidden_id_);
  }
  groups.AddPosition("kicker", pass_tactic_.getKickerAssignmentPoint(world), disallowed_strikers);
  groups.AddPosition("receiver", pass_tactic_.getReceiverAssignmentPoint(world));
  const auto enough_bots_for_defense = available_robots.size() >= 4;
  if (enough_bots_for_defense) {
    groups.AddGroup("defense", defense_tactic_.getAssignmentPoints(world));
  }
  const auto enough_bots_for_idler = available_robots.size() >= 5;
  if (enough_bots_for_idler) {
    groups.AddPosition("idler", idler_skill_.GetAssignmentPoint(world));
  }

  const auto assignments = play_helpers::assignGroups(available_robots, groups);

  if (enough_bots_for_defense) {
    defense_tactic_.runFrame(
      world, assignments.GetGroupFilledAssignments("defense"),
      motion_commands);
  }

  const auto maybe_kicker = assignments.GetPositionAssignment("kicker");
  const auto maybe_receiver = assignments.GetPositionAssignment("receiver");
  if (maybe_kicker && maybe_receiver) {
    auto & kicker_command =
      *(motion_commands[maybe_kicker->id] = ateam_msgs::msg::RobotMotionCommand{});
    auto & receiver_command =
      *(motion_commands[maybe_receiver->id] = ateam_msgs::msg::RobotMotionCommand{});

    if (CGAL::squared_distance(world.ball.pos, maybe_kicker->pos) < 0.5) {
      started_ = true;
    }

    getPlayInfo()["kicker"] = maybe_kicker->id;
    getPlayInfo()["receiver"] = maybe_receiver->id;

    pass_tactic_.runFrame(world, *maybe_kicker, *maybe_receiver, kicker_command, receiver_command);
  }

  if (enough_bots_for_idler) {
    assignments.RunPositionIfAssigned(
      "idler", [this, &world, &motion_commands](const Robot & robot) {
        motion_commands[robot.id] = idler_skill_.RunFrame(world, robot);
      });
  }

  return motion_commands;
}


play_helpers::lanes::Lane PassToSegmentPlay::getIdleLane(
  const World & world,
  const ateam_geometry::Segment & target)
{
  std::vector<play_helpers::lanes::Lane> lanes = {
    play_helpers::lanes::Lane::Left,
    play_helpers::lanes::Lane::Center,
    play_helpers::lanes::Lane::Right,
  };

  const auto target_midpoint = CGAL::midpoint(target);
  lanes.erase(
    std::remove_if(
      lanes.begin(), lanes.end(), [&target_midpoint, &world](const auto & lane) {
        return play_helpers::lanes::IsPointInLane(world, target_midpoint, lane) ||
               play_helpers::lanes::IsBallInLane(world, lane);
      }), lanes.end());

  if (lanes.empty()) {
    // Fallback to center lane
    return play_helpers::lanes::Lane::Center;
  }

  return lanes.front();
}

}  // namespace ateam_kenobi::plays
