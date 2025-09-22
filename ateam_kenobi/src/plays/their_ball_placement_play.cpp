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


#include "their_ball_placement_play.hpp"
#include <angles/angles.h>
#include <algorithm>
#include <vector>
#include <ateam_common/robot_constants.hpp>
#include "core/play_helpers/available_robots.hpp"
#include <ateam_geometry/ateam_geometry.hpp>

namespace ateam_kenobi::plays
{

TheirBallPlacementPlay::TheirBallPlacementPlay(stp::Options stp_options)
: stp::Play(kPlayName, stp_options),
  multi_move_to_(createChild<tactics::MultiMoveTo>("multi_move_to"))
{
}

stp::PlayScore TheirBallPlacementPlay::getScore(const World & world)
{
  const auto & cmd = world.referee_info.running_command;
  if (cmd == ateam_common::GameCommand::BallPlacementTheirs) {
    return stp::PlayScore::Max();
  }
  return stp::PlayScore::NaN();
}

std::array<std::optional<RobotCommand>, 16> TheirBallPlacementPlay::runFrame(
  const World & world)
{
  std::array<std::optional<RobotCommand>, 16> maybe_motion_commands;

  auto available_robots = play_helpers::getAvailableRobots(world);

  const ateam_geometry::Point placement_point = [this, &world]() {
      if (world.referee_info.designated_position.has_value()) {
        return world.referee_info.designated_position.value();
      } else {
        RCLCPP_WARN(
        getLogger(),
        "No designated position set in referee info, using ball position.");
        return world.ball.pos;
      }
    }();


  DrawKeepoutArea(world.ball.pos, placement_point);

  const auto partition_iter = std::partition(available_robots.begin(), available_robots.end(),
      [this, &world, &placement_point](const Robot & robot) {
        return shouldRobotMove(world, placement_point, robot);
    });

  const std::vector<Robot> robots_to_move(available_robots.begin(), partition_iter);
  const std::vector<Robot> robots_to_stay(partition_iter, available_robots.end());

  for(const auto & robot : robots_to_move) {
    getPlayInfo()["Robots"][std::to_string(robot.id)] = "MOVING";
  }
  for(const auto & robot : robots_to_stay) {
    getPlayInfo()["Robots"][std::to_string(robot.id)] = "-";
  }

  std::vector<ateam_geometry::Point> target_points;
  std::transform(robots_to_move.begin(), robots_to_move.end(),
    std::back_inserter(target_points),
    [this, &world, &placement_point](const Robot & robot) {
      return getTargetPoint(world, placement_point, robot);
    });

  multi_move_to_.SetTargetPoints(target_points);
  multi_move_to_.SetFacePoint(world.ball.pos);
  multi_move_to_.RunFrame(robots_to_move, maybe_motion_commands);

  return maybe_motion_commands;
}

void TheirBallPlacementPlay::DrawKeepoutArea(
  const ateam_geometry::Point & ball_pos,
  const ateam_geometry::Point & placement_point)
{
  const auto point_to_ball = ball_pos - placement_point;
  const auto angle = std::atan2(point_to_ball.y(), point_to_ball.x());

  auto & overlays = getOverlays();

  const auto keepout_radius = 0.5;
  const ateam_geometry::Vector pos_offset{keepout_radius * std::cos(angle + M_PI_2),
    keepout_radius * std::sin(angle + M_PI_2)};
  const ateam_geometry::Vector neg_offset{keepout_radius * std::cos(angle - M_PI_2),
    keepout_radius * std::sin(angle - M_PI_2)};

  const ateam_geometry::Arc ball_side_arc{ball_pos, keepout_radius, neg_offset.direction(),
    pos_offset.direction()};
  const ateam_geometry::Arc point_side_arc{placement_point, keepout_radius, pos_offset.direction(),
    neg_offset.direction()};
  const ateam_geometry::Segment pos_segment{placement_point + pos_offset, ball_pos + pos_offset};
  const ateam_geometry::Segment neg_segment{placement_point + neg_offset, ball_pos + neg_offset};


  overlays.drawArc("placement_avoid_ball_arc", ball_side_arc, "Red");
  overlays.drawArc("placement_avoid_place_arc", point_side_arc, "Red");
  overlays.drawLine("placement_avoid_pos_line", {pos_segment.source(), pos_segment.target()},
      "Red");
  overlays.drawLine("placement_avoid_neg_line", {neg_segment.source(), neg_segment.target()},
      "Red");
}

bool TheirBallPlacementPlay::shouldRobotMove(
  const World & world,
  const ateam_geometry::Point & placement_point, const Robot & robot)
{
  const auto placement_segment = ateam_geometry::Segment(placement_point, world.ball.pos);
  const auto nearest_point = ateam_geometry::nearestPointOnSegment(placement_segment, robot.pos);
  return ateam_geometry::norm(robot.pos - nearest_point) < 0.6 + kRobotRadius;
}

ateam_geometry::Point TheirBallPlacementPlay::getTargetPoint(
  const World & world,
  const ateam_geometry::Point & placement_point, const Robot & robot)
{
  const auto placement_segment = ateam_geometry::Segment(placement_point, world.ball.pos);
  const auto nearest_point = ateam_geometry::nearestPointOnSegment(placement_segment, robot.pos);

  const auto point_to_ball = world.ball.pos - placement_point;
  const auto angle = std::atan2(point_to_ball.y(), point_to_ball.x());

  auto target_position = nearest_point +
    0.7 * ateam_geometry::Vector(std::cos(angle + M_PI / 2), std::sin(angle + M_PI / 2));

  const auto alternate_position = nearest_point +
    0.7 * ateam_geometry::Vector(std::cos(angle - M_PI / 2), std::sin(angle - M_PI / 2));

  const auto offset = kRobotRadius * 0.95;
  const auto x_bound = (world.field.field_length / 2.0) + world.field.boundary_width - offset;
  const auto y_bound = (world.field.field_width / 2.0) + world.field.boundary_width - offset;
  ateam_geometry::Rectangle pathable_region(ateam_geometry::Point(-x_bound, -y_bound),
    ateam_geometry::Point(x_bound, y_bound));

  if (!CGAL::do_intersect(target_position, pathable_region)) {
    target_position = alternate_position;
  } else if (!CGAL::do_intersect(alternate_position, pathable_region)) {
        // Stick with target_position
  } else {
        // Use the shorter path
    if (ateam_geometry::norm(target_position - robot.pos) >
      ateam_geometry::norm(alternate_position - robot.pos))
    {
      target_position = alternate_position;
    }
  }
  return target_position;
}

}  // namespace ateam_kenobi::plays
