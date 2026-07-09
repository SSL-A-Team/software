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

#include "possession.hpp"
#include <ranges>
#include <algorithm>
#include <limits>
#include <ateam_common/robot_constants.hpp>
#include "core/types/state_types.hpp"

namespace ateam_kenobi::play_helpers
{

PossessionResult WhoHasPossession(const World & world)
{
  const ateam_geometry::Rectangle our_defense_area {
    -1 * (world.field.field_length / 2.0),
    -((world.field.defense_area_width / 2.0) + kRobotRadius / 2),
    -1 * ((world.field.field_length / 2.0) - world.field.defense_area_depth - kRobotRadius / 2),
    (world.field.defense_area_width / 2.0) - kRobotRadius / 2
  };
  if (ateam_geometry::doIntersect(our_defense_area, world.ball.pos)) {
    return PossessionResult::Ours;
  }

  const auto possession_threshold = 0.01 + kRobotRadius + kBallRadius;
  const auto weak_possession_threshold = 0.5 + kRobotRadius + kBallRadius;
  const auto possession_threhold_sq = possession_threshold * possession_threshold;
  const auto weak_possession_threshold_sq = weak_possession_threshold * weak_possession_threshold;
  const auto & ball_pos = world.ball.pos;

  const double possession_angle = 0.35;

  double closest_our_bot_sq_distance = std::numeric_limits<double>::infinity();
  double closest_their_bot_sq_distance = std::numeric_limits<double>::infinity();

  bool we_have_control_of_ball = false;
  bool they_have_control_of_ball = false;

  for (const auto & robot : world.our_robots) {
    if (!robot.visible) {
      continue;
    }
    const auto distance = CGAL::squared_distance(ball_pos, robot.pos);
    closest_our_bot_sq_distance = std::min(closest_our_bot_sq_distance, distance);

    const double angle = ateam_geometry::ToHeading(ball_pos - robot.pos);
    if (distance <= possession_threshold &&
      abs(angles::shortest_angular_distance(angle, robot.theta)) < possession_angle)
    {
      we_have_control_of_ball = true;
    }
  }

  for (const auto & robot : world.their_robots) {
    if (!robot.visible) {
      continue;
    }
    const auto distance = CGAL::squared_distance(ball_pos, robot.pos);
    closest_their_bot_sq_distance = std::min(closest_their_bot_sq_distance, distance);

    const double angle = ateam_geometry::ToHeading(ball_pos - robot.pos);
    if (distance <= possession_threshold &&
      abs(angles::shortest_angular_distance(angle, robot.theta)) < possession_angle)
    {
      they_have_control_of_ball = true;
    }
  }

  const auto we_are_closer = closest_our_bot_sq_distance < closest_their_bot_sq_distance;
  const auto they_are_closer = closest_our_bot_sq_distance > closest_their_bot_sq_distance;
  const auto we_are_close_enough = closest_our_bot_sq_distance < possession_threhold_sq;
  const auto they_are_close_enough = closest_their_bot_sq_distance < possession_threhold_sq;
  const auto we_are_weakly_close_enough = closest_our_bot_sq_distance <
    weak_possession_threshold_sq;
  const auto they_are_weakly_close_enough = closest_their_bot_sq_distance <
    weak_possession_threshold_sq;

  if (we_have_control_of_ball && they_have_control_of_ball) {
    return PossessionResult::Neither;
  } else if (we_have_control_of_ball) {
    return PossessionResult::Ours;
  } else if (they_have_control_of_ball) {
    return PossessionResult::Theirs;
  } else if (we_are_closer && we_are_close_enough) {
    return PossessionResult::Ours;
  } else if (they_are_closer && they_are_close_enough) {
    return PossessionResult::Theirs;
  } else if (we_are_closer && we_are_weakly_close_enough) {
    return PossessionResult::OursWeak;
  } else if (they_are_closer && they_are_weakly_close_enough) {
    return PossessionResult::TheirsWeak;
  } else {
    return PossessionResult::Neither;
  }
}

}  // namespace ateam_kenobi::play_helpers
