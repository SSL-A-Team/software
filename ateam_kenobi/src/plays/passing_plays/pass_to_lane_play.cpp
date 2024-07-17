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

#include "pass_to_lane_play.hpp"
#include <algorithm>
#include <limits>
#include "play_helpers/available_robots.hpp"
#include "play_helpers/window_evaluation.hpp"
#include "play_helpers/shot_evaluation.hpp"
#include "play_helpers/possession.hpp"

namespace ateam_kenobi::plays
{

PassToLanePlay::PassToLanePlay(
  stp::Options stp_options, play_helpers::lanes::Lane lane,
  PassDirection direction)
: PassToSegmentPlay(stp_options, std::bind_front(&PassToLanePlay::getTargetSegment, this)),
  lane_(lane),
  direction_(direction)
{
}

stp::PlayScore PassToLanePlay::getScore(const World & world)
{
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

  if (play_helpers::lanes::IsBallInLane(world, lane_)) {
    return stp::PlayScore::Min();
  }

  const auto target = getTargetSegment(world);
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

ateam_geometry::Segment PassToLanePlay::getTargetSegment(const World & world)
{
  const auto lane_segment = play_helpers::lanes::GetLaneLongitudinalMidSegment(world, lane_);
  const auto max_center_x = (world.field.field_length / 2.0) - world.field.defense_area_depth;
  const auto max_x = lane_ ==
    play_helpers::lanes::Lane::Center ? max_center_x : std::numeric_limits<double>::infinity();
  const auto ball_x = std::min(world.ball.pos.x(), max_x);
  if (direction_ == PassDirection::Forward) {
    return ateam_geometry::Segment{
      ateam_geometry::Point{std::clamp(lane_segment.source().x(), ball_x, max_x),
        lane_segment.source().y()},
      ateam_geometry::Point{std::clamp(lane_segment.target().x(), ball_x, max_x),
        lane_segment.target().y()}
    };
  } else {
    return ateam_geometry::Segment{
      ateam_geometry::Point{std::min(lane_segment.source().x(), ball_x),
        lane_segment.source().y()},
      ateam_geometry::Point{std::min(lane_segment.target().x(), ball_x),
        lane_segment.target().y()}
    };
  }
}

}  // namespace ateam_kenobi::plays
