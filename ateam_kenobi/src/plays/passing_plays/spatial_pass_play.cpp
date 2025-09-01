// Copyright 2025 A Team
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

#include "spatial_pass_play.hpp"
#include <algorithm>
#include <string>
#include <vector>
#include <ateam_common/robot_constants.hpp>
#include <ateam_geometry/types.hpp>
#include "core/play_helpers/available_robots.hpp"
#include "core/play_helpers/robot_assignment.hpp"
#include "core/play_helpers/window_evaluation.hpp"
#include "core/play_helpers/possession.hpp"

namespace ateam_kenobi::plays
{

SpatialPassPlay::SpatialPassPlay(stp::Options stp_options)
: stp::Play(kPlayName, stp_options),
  defense_tactic_(createChild<tactics::StandardDefense>("defense")),
  pass_tactic_(createChild<tactics::Pass>("pass")),
  idler_skill_(createChild<skills::LaneIdler>("idler"))
{
  getParamInterface().declareParameter("show_heatmap", false);
}


stp::PlayScore SpatialPassPlay::getScore(const World & world)
{
  if (!world.in_play && (
      world.referee_info.prev_command == ateam_common::GameCommand::PrepareKickoffOurs ||
      world.referee_info.prev_command == ateam_common::GameCommand::PrepareKickoffTheirs))
  {
    return stp::PlayScore::NaN();
  }
  if (!world.in_play &&
    world.referee_info.running_command != ateam_common::GameCommand::ForceStart &&
    world.referee_info.running_command != ateam_common::GameCommand::NormalStart &&
    world.referee_info.running_command != ateam_common::GameCommand::DirectFreeOurs)
  {
    return stp::PlayScore::NaN();
  }
  if(play_helpers::WhoHasPossession(world) == play_helpers::PossessionResult::Theirs) {
    return stp::PlayScore::Min();
  }

  ateam_geometry::Point pass_target;

  if(started_) {
    pass_target = target_;
  } else {
    const auto spatial_target =
      world.spatial_evaluator->GetMaxLocation(ateam_spatial::MapId::ReceiverPositionQuality);
    pass_target = ateam_geometry::Point{spatial_target.x, spatial_target.y};
  }

  const auto their_robots = play_helpers::getVisibleRobots(world.their_robots);
  const ateam_geometry::Segment goal_segment(
    ateam_geometry::Point(world.field.field_length / 2.0, -world.field.goal_width),
    ateam_geometry::Point(world.field.field_length / 2.0, world.field.goal_width)
  );
  const auto windows = play_helpers::window_evaluation::getWindows(goal_segment, pass_target,
      their_robots);
  const auto largest_window = play_helpers::window_evaluation::getLargestWindow(windows);
  if(!largest_window) {
    return stp::PlayScore::Min();
  }
  return std::min(90.0,
      stp::PlayScore::Max() * (largest_window->squared_length() / goal_segment.squared_length()));
}

stp::PlayCompletionState SpatialPassPlay::getCompletionState()
{
  if (!started_) {
    return stp::PlayCompletionState::NotApplicable;
  }
  if (!pass_tactic_.isDone()) {
    return stp::PlayCompletionState::Busy;
  }
  return stp::PlayCompletionState::Done;
}

void SpatialPassPlay::reset()
{
  defense_tactic_.reset();
  pass_tactic_.reset();
  idler_skill_.Reset();
  started_ = false;
  target_ = ateam_geometry::Point{};
}

std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
  16> SpatialPassPlay::runFrame(const World & world)
{
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> motion_commands;

  const auto spatial_map_id = ateam_spatial::MapId::ReceiverPositionQuality;
  const auto is_pass_blocked = world.spatial_evaluator->GetValueAtLocation(spatial_map_id,
      ateam_spatial::Point(target_.x(), target_.y())) < 1;
  if(!started_ || is_pass_blocked) {
    // TODO(barulicm): need to granularize "started" state to not change target while ball is moving
    const auto spatial_target = world.spatial_evaluator->GetMaxLocation(spatial_map_id);
    target_ = ateam_geometry::Point{spatial_target.x, spatial_target.y};
  }

  getOverlays().drawCircle("target", ateam_geometry::makeCircle(target_, kRobotRadius), "grey");
  const auto half_field_length = (world.field.field_length / 2.0) + world.field.boundary_width;
  const auto half_field_width = (world.field.field_width / 2.0) + world.field.boundary_width;
  ateam_geometry::Rectangle bounds {
    ateam_geometry::Point{-half_field_length, -half_field_width},
    ateam_geometry::Point{half_field_length, half_field_width}
  };

  if(getParamInterface().getParameter<bool>("show_heatmap")) {
    world.spatial_evaluator->RenderMapBuffer(spatial_map_id, spatial_map_rendering_data_);
    const auto spatial_settings = world.spatial_evaluator->GetSettings();
    getOverlays().drawHeatmap("heatmap", bounds, spatial_map_rendering_data_,
        spatial_settings.width, spatial_settings.height);
  }

  pass_tactic_.setTarget(target_);

  idler_skill_.SetLane(getIdleLane(world));

  auto available_robots = play_helpers::getAvailableRobots(world);
  play_helpers::removeGoalie(available_robots, world);

  play_helpers::GroupAssignmentSet groups;

  std::vector<int> disallowed_strikers;
  if (world.double_touch_forbidden_id_) {
    disallowed_strikers.push_back(*world.double_touch_forbidden_id_);
  }
  groups.AddPosition("kicker", pass_tactic_.getKickerAssignmentPoint(world), disallowed_strikers);
  groups.AddPosition("receiver", pass_tactic_.getReceiverAssignmentPoint());
  const auto enough_bots_for_defense = available_robots.size() >= 4;
  if (enough_bots_for_defense) {
    groups.AddGroup("defense", defense_tactic_.getAssignmentPoints(world));
  }
  const auto enough_bots_for_idler = available_robots.size() >= 5;
  if (enough_bots_for_idler) {
    groups.AddPosition("idler", idler_skill_.GetAssignmentPoint(world));
  }

  const auto assignments = play_helpers::assignGroups(available_robots, groups);

  defense_tactic_.runFrame(world, assignments.GetGroupFilledAssignmentsOrEmpty("defense"),
      motion_commands);

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

    pass_tactic_.runFrame(world, *maybe_kicker, *maybe_receiver, kicker_command, receiver_command);

    ForwardPlayInfo(pass_tactic_);
  }

  std::string play_state;
  if(!started_) {
    play_state = "Not Started";
  } else if(!pass_tactic_.isDone()) {
    play_state = "Passing";
  } else {
    play_state = "Done";
  }
  getPlayInfo()["play state"] = play_state;

  if (enough_bots_for_idler) {
    assignments.RunPositionIfAssigned(
      "idler", [this, &world, &motion_commands](const Robot & robot) {
        motion_commands[robot.id] = idler_skill_.RunFrame(world, robot);
      });
  }

  return motion_commands;
}


play_helpers::lanes::Lane SpatialPassPlay::getIdleLane(const World & world)
{
  std::vector<play_helpers::lanes::Lane> lanes = {
    play_helpers::lanes::Lane::Left,
    play_helpers::lanes::Lane::Center,
    play_helpers::lanes::Lane::Right,
  };
  for(const auto lane : lanes) {
    if(!play_helpers::lanes::IsPointInLane(world, target_, lane) &&
      !play_helpers::lanes::IsBallInLane(world, lane))
    {
      return lane;
    }
  }
  return play_helpers::lanes::Lane::Center;
}

}  // namespace ateam_kenobi::plays
