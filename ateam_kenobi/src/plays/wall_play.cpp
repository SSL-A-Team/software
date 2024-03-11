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

#include <CGAL/point_generators_2.h>

#include <ateam_common/robot_constants.hpp>
#include "wall_play.hpp"
#include "robot_assignment.hpp"
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
    points_on_segment.push_back(segment.vertex(0));
  }

  auto source = segment.vertex(0);
  auto target = segment.vertex(1);

  ateam_geometry::Point spacing = ateam_geometry::Point(
    // source.x() + target.x() / (num_points - 1),
    0,
    CGAL::approximate_sqrt((source - target).squared_length()) / (num_points - 1)
  );

  for (int i = 0; i < num_points; ++i) {
    auto segment_point = ateam_geometry::Point(
      // source.x() + (spacing.x() * i),
      target.x(),
      target.y() + (spacing.y() * i)
      // both of these were source in the last nothing else changed
    );

    points_on_segment.push_back(segment_point);
    // WHAT THE ACTUAL FUCK
  }
  return points_on_segment;
}


WallPlay::WallPlay()
: BasePlay("WallPlay"),
  goalie_skill_(getOverlays().getChild("goalie"))
{
  play_helpers::EasyMoveTo::CreateArray(easy_move_tos_, getOverlays().getChild("EasyMoveTo"));
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
  auto current_available_robots = play_helpers::getAvailableRobots(world);
  play_helpers::removeGoalie(current_available_robots, world);

  if (current_available_robots.empty()) {
    return maybe_motion_commands;
  }

  ateam_geometry::Segment wall_line = ateam_geometry::Segment(
    ateam_geometry::Point(-3, 0.25 * current_available_robots.size() / 2.0),
    ateam_geometry::Point(-3, -0.25 * current_available_robots.size() / 2.0)
  );

  std::vector<ateam_geometry::Point> positions_to_assign =
    get_equally_spaced_points_on_segment(wall_line, current_available_robots.size());

  const auto & robot_assignments = robot_assignment::assign(
    current_available_robots,
    positions_to_assign);
  for (const auto [robot_id, pos_ind] : robot_assignments) {
    const auto & maybe_assigned_robot = world.our_robots.at(robot_id);

    if (!maybe_assigned_robot) {
      // TODO(barulicm): log this?
      continue;
    }

    const Robot & robot = maybe_assigned_robot.value();

    auto & easy_move_to = easy_move_tos_.at(robot_id);

    const auto & target_position = positions_to_assign.at(pos_ind);

    auto viz_circle = ateam_geometry::makeCircle(target_position, kRobotRadius);
    getOverlays().drawCircle(
      "destination_" + std::to_string(
        robot_id), viz_circle, "blue", "transparent");

    easy_move_to.setTargetPosition(target_position);
    easy_move_to.face_absolute(0);   // face away from our goal

    maybe_motion_commands.at(robot_id) = easy_move_to.runFrame(robot, world);
  }

  goalie_skill_.runFrame(world, maybe_motion_commands);

  return maybe_motion_commands;
}

}  // namespace ateam_kenobi::plays
