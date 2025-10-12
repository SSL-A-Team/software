// Copyright 2023 A Team
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


#include "wall_play.hpp"
#include <CGAL/point_generators_2.h>
#include <limits>
#include <ateam_common/robot_constants.hpp>
#include "core/play_helpers/robot_assignment.hpp"
#include "core/types.hpp"
#include "skills/goalie.hpp"
#include "core/play_helpers/available_robots.hpp"
namespace ateam_kenobi::plays
{
std::vector<ateam_geometry::Point> get_equally_spaced_points_on_segment(
  ateam_geometry::Segment & segment, int num_points)
{
  std::vector<ateam_geometry::Point> points_on_segment;

  if (num_points == 1) {
    points_on_segment.push_back(CGAL::midpoint(segment));
  }

  auto source = segment.vertex(0);
  auto target = segment.vertex(1);

  ateam_geometry::Vector spacing = (target - source) / (num_points - 1);

  auto segment_point = source;
  for (int i = 0; i < num_points; ++i) {
    points_on_segment.push_back(segment_point);
    segment_point += spacing;
  }
  return points_on_segment;
}

WallPlay::WallPlay(stp::Options stp_options)
: stp::Play(kPlayName, stp_options),
  goalie_skill_(createChild<skills::Goalie>("goalie")),
  multi_move_to_(createChild<tactics::MultiMoveTo>("multi_move_to"))
{
}

stp::PlayScore WallPlay::getScore(const World &)
{
  return stp::PlayScore::Min();
}

void WallPlay::reset()
{
  goalie_skill_.reset();
}

std::array<std::optional<RobotCommand>, 16> WallPlay::runFrame(
  const World & world)
{
  std::array<std::optional<RobotCommand>, 16> commands;

  goalie_skill_.runFrame(world, commands);

  auto current_available_robots = play_helpers::getAvailableRobots(world);
  play_helpers::removeGoalie(current_available_robots, world);

  const auto & our_defense_corners = world.field.ours.defense_area_corners;
  const double defense_area_depth =
    std::abs(
    CGAL::right_vertex_2(
      our_defense_corners.begin(),
      our_defense_corners.end())->x() -
    CGAL::left_vertex_2(our_defense_corners.begin(), our_defense_corners.end())->x());
  const double wall_x = ((-1 * world.field.field_length) / 2.0) + defense_area_depth + 0.2;

  ateam_geometry::Segment wall_line = ateam_geometry::Segment(
    ateam_geometry::Point(wall_x, 0.25 * current_available_robots.size() / 2.0),
    ateam_geometry::Point(wall_x, -0.25 * current_available_robots.size() / 2.0)
  );

  std::vector<ateam_geometry::Point> positions_to_assign =
    get_equally_spaced_points_on_segment(wall_line, current_available_robots.size());

  const auto robot_assignments = play_helpers::assignRobots(
    current_available_robots,
    positions_to_assign);

  multi_move_to_.SetTargetPoints(positions_to_assign);
  multi_move_to_.SetFacePoint(world.ball.pos);
  multi_move_to_.RunFrame(robot_assignments, commands);

  return commands;
}

}  // namespace ateam_kenobi::plays
