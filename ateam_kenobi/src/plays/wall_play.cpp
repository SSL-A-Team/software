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
#include "play_helpers/robot_assignment.hpp"
#include "types/robot.hpp"
#include "skills/goalie.hpp"
#include "play_helpers/available_robots.hpp"
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
  easy_move_tos_(createIndexedChildren<play_helpers::EasyMoveTo>("EasyMoveTo")),
  goalie_skill_(createChild<skills::Goalie>("goalie"))
{
}

double WallPlay::getScore(const World & world)
{
  switch (world.referee_info.running_command) {
    case ateam_common::GameCommand::PrepareKickoffTheirs:
      return std::numeric_limits<double>::max();
    case ateam_common::GameCommand::DirectFreeTheirs:
      return world.in_play ? std::numeric_limits<double>::lowest() : std::numeric_limits<double>::
             max();
    case ateam_common::GameCommand::NormalStart:
      {
        if (world.in_play) {
          return std::numeric_limits<double>::lowest();
        }
        switch (world.referee_info.prev_command) {
          case ateam_common::GameCommand::PrepareKickoffTheirs:
          case ateam_common::GameCommand::DirectFreeTheirs:
            return std::numeric_limits<double>::max();
          default:
            return std::numeric_limits<double>::lowest();
        }
      }
    default:
      return std::numeric_limits<double>::lowest();
  }
}

void WallPlay::reset()
{
  for (auto & move_to : easy_move_tos_) {
    move_to.reset();
  }
  goalie_skill_.reset();
}

std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> WallPlay::runFrame(
  const World & world)
{
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> maybe_motion_commands;

  goalie_skill_.runFrame(world, maybe_motion_commands);

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

  for (auto ind = 0ul; ind < robot_assignments.size(); ++ind) {
    const auto & maybe_robot = robot_assignments[ind];
    if (!maybe_robot) {
      continue;
    }
    const auto & robot = *maybe_robot;
    const auto & target_position = positions_to_assign[ind];

    auto & easy_move_to = easy_move_tos_.at(robot.id);

    auto viz_circle = ateam_geometry::makeCircle(target_position, kRobotRadius);
    getOverlays().drawCircle(
      "destination_" + std::to_string(
        robot.id), viz_circle, "blue", "transparent");

    easy_move_to.setTargetPosition(target_position);
    easy_move_to.face_point(world.ball.pos);

    maybe_motion_commands.at(robot.id) = easy_move_to.runFrame(robot, world);
  }

  return maybe_motion_commands;
}

}  // namespace ateam_kenobi::plays
