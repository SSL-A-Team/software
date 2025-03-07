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

#include "defenders.hpp"
#include <algorithm>
#include <ateam_common/robot_constants.hpp>
#include <ateam_geometry/ateam_geometry.hpp>
#include "core/play_helpers/available_robots.hpp"

namespace ateam_kenobi::tactics
{

Defenders::Defenders(stp::Options stp_options)
: stp::Tactic(stp_options)
{
  createIndexedChildren<play_helpers::EasyMoveTo>(easy_move_tos_, "EasyMoveTo");
}

void Defenders::reset()
{
  std::ranges::for_each(easy_move_tos_, std::mem_fn(&play_helpers::EasyMoveTo::reset));
}

std::vector<ateam_geometry::Point> Defenders::getAssignmentPoints(const World & world)
{
  return getDefenderPoints(world);
}

void Defenders::runFrame(
  const World & world,
  const std::vector<Robot> & robots,
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> & motion_commands)
{
  const auto defender_points = getDefenderPoints(world);
  const auto num_defenders = std::min(defender_points.size(), robots.size());

  for (auto i = 0ul; i < num_defenders; ++i) {
    const auto & robot = robots[i];
    const auto & defender_point = defender_points[i];
    auto & emt = easy_move_tos_[robot.id];
    emt.setTargetPosition(defender_point);
    emt.face_point(world.ball.pos);
    path_planning::PlannerOptions planner_options;
    planner_options.avoid_ball = false;
    planner_options.footprint_inflation = 0.03;
    emt.setPlannerOptions(planner_options);
    motion_commands[robot.id] = emt.runFrame(robot, world);
  }

  drawDefenseSegments(world);
}

std::vector<ateam_geometry::Point> Defenders::getDefenderPoints(const World & world)
{
  const auto first_position = getBallBlockPoint(world);
  const auto visible_oponent_robots = play_helpers::getVisibleRobots(world.their_robots);
  ateam_geometry::Point second_position;
  const auto min_opponent_bots = 2;
  if (visible_oponent_robots.size() >= min_opponent_bots) {
    second_position = getPassBlockPoint(world);
  }

  if (visible_oponent_robots.size() < min_opponent_bots ||
    CGAL::squared_distance(second_position, first_position) < kRobotDiameter * kRobotDiameter)
  {
    second_position = getAdjacentBlockPoint(world, first_position);
  }
  return {first_position, second_position};
}

ateam_geometry::Point Defenders::getBallBlockPoint(const World & world)
{
  const auto defense_segments = getDefenseSegments(world);
  std::vector<ateam_geometry::Point> closest_points;
  std::ranges::transform(
    defense_segments, std::back_inserter(closest_points), [&world](const auto & segment) {
      return ateam_geometry::nearestPointOnSegment(segment, world.ball.pos);
    });

  std::vector<double> squared_distances;
  std::ranges::transform(
    closest_points, std::back_inserter(squared_distances), [&world](const auto & pt) {
      return CGAL::squared_distance(pt, world.ball.pos);
    });

  const auto closest_iter = std::ranges::min_element(squared_distances);

  const auto closest_index = std::distance(squared_distances.begin(), closest_iter);

  return closest_points[closest_index];
}

ateam_geometry::Point Defenders::getPassBlockPoint(const World & world)
{
  const auto segments = getDefenseSegments(world);

  ateam_geometry::Point block_point = CGAL::midpoint(segments.front());

  auto their_bots = play_helpers::getVisibleRobots(world.their_robots);

  if (their_bots.size() < 2) {
    return block_point;
  }

  auto by_dist_to_ball = [&world](const Robot & a, const Robot & b) {
      return CGAL::compare_distance_to_point(world.ball.pos, a.pos, b.pos) == CGAL::SMALLER;
    };

  std::ranges::sort(their_bots, by_dist_to_ball);

  const auto pass_candidate_bot = their_bots[1];

  const ateam_geometry::Segment pass_candidate_line(pass_candidate_bot.pos,
    ateam_geometry::Point(-world.field.field_length / 2.0, 0));

  for (const auto & seg : segments) {
    const auto intersection = ateam_geometry::intersection(seg, pass_candidate_line);
    if (!intersection) {
      continue;
    }
    if (!std::holds_alternative<ateam_geometry::Point>(*intersection)) {
      continue;
    }
    block_point = std::get<ateam_geometry::Point>(*intersection);
    break;
  }

  return block_point;
}

ateam_geometry::Point Defenders::getAdjacentBlockPoint(
  const World & world,
  const ateam_geometry::Point & other_block_point)
{
  const auto defense_segments = getDefenseSegments(world);

  auto by_dist = [other_block_point](const auto & a, const auto & b) {
      return CGAL::squared_distance(a, other_block_point) < CGAL::squared_distance(
        b,
        other_block_point);
    };

  auto closest_seg = *std::ranges::min_element(defense_segments, by_dist);

  return other_block_point +
         (ateam_geometry::normalize(closest_seg.to_vector()) * (kRobotDiameter + 0.05));
}


std::vector<ateam_geometry::Segment> Defenders::getDefenseSegments(const World & world)
{
  std::vector<ateam_geometry::Segment> segments;

  const auto margin = 0.05;

  const auto pos_y_extent = (world.field.defense_area_width / 2.0) + kRobotRadius + margin;
  const auto neg_y_extent = -pos_y_extent;
  const auto x_front = (-world.field.field_length / 2.0) + world.field.defense_area_depth +
    kRobotRadius +
    margin;
  const auto x_back = -world.field.field_length / 2.0;

  segments.push_back(
    ateam_geometry::Segment(
      ateam_geometry::Point(x_front, neg_y_extent),
      ateam_geometry::Point(x_front, pos_y_extent)
  ));

  segments.push_back(
    ateam_geometry::Segment(
      ateam_geometry::Point(x_back, neg_y_extent),
      ateam_geometry::Point(x_front, neg_y_extent)
  ));

  segments.push_back(
    ateam_geometry::Segment(
      ateam_geometry::Point(x_back, pos_y_extent),
      ateam_geometry::Point(x_front, pos_y_extent)
  ));

  return segments;
}

void Defenders::drawDefenseSegments(const World & world)
{
  const auto segments = getDefenseSegments(world);
  auto i = 0;
  for (const ateam_geometry::Segment & seg : segments) {
    getOverlays().drawLine(
      "defense_line_" + std::to_string(i), {seg.source(), seg.target()},
      "blue");
    i++;
  }
}

}  // namespace ateam_kenobi::tactics
