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

#include "defense_area_enforcement.hpp"
#include <ateam_geometry/ateam_geometry.hpp>
#include <ateam_common/robot_constants.hpp>

namespace ateam_kenobi::defense_area_enforcement
{

void EnforceDefenseAreaKeepout(
  const World & world,
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> & motion_commands)
{
  for (auto robot_id = 0ul; robot_id < motion_commands.size(); ++robot_id) {
    if (robot_id == static_cast<std::size_t>(world.referee_info.our_goalie_id)) {
      continue;
    }
    auto & maybe_command = motion_commands[robot_id];
    if (!maybe_command) {
      continue;
    }
    auto & command = *maybe_command;
    if (WouldVelocityCauseCollision(world, robot_id, command)) {
      command.twist.linear.x = 0.0;
      command.twist.linear.y = 0.0;
    }
  }
}

bool WouldVelocityCauseCollision(
  const World & world, const int robot_id,
  const ateam_msgs::msg::RobotMotionCommand & motion_command)
{
  const ateam_geometry::Rectangle our_defense_area{
    *CGAL::top_vertex_2(
      world.field.ours.defense_area_corners.begin(),
      world.field.ours.defense_area_corners.end()),
    *CGAL::bottom_vertex_2(
      world.field.ours.defense_area_corners.begin(),
      world.field.ours.defense_area_corners.end())
  };
  const ateam_geometry::Rectangle their_defense_area {
    *CGAL::top_vertex_2(
      world.field.theirs.defense_area_corners.begin(),
      world.field.theirs.defense_area_corners.end()),
    *CGAL::bottom_vertex_2(
      world.field.theirs.defense_area_corners.begin(),
      world.field.theirs.defense_area_corners.end())
  };

  const double delta_t = 0.01;

  const ateam_geometry::Vector velocity{motion_command.twist.linear.x,
    motion_command.twist.linear.y};

  const ateam_geometry::Vector displacement = velocity * delta_t;

  const auto & robot = world.our_robots[robot_id];

  const ateam_geometry::Point new_position = robot.pos + displacement;

  const ateam_geometry::Disk robot_footprint = ateam_geometry::makeDisk(new_position, kRobotRadius);

  if (ateam_geometry::doIntersect(robot_footprint, our_defense_area)) {
    return !IsRobotEscapingDefenseArea(robot.pos, new_position, our_defense_area);
  }

  if (ateam_geometry::doIntersect(robot_footprint, their_defense_area)) {
    return !IsRobotEscapingDefenseArea(robot.pos, new_position, their_defense_area);
  }

  return false;
}

bool IsRobotEscapingDefenseArea(
  const ateam_geometry::Point & position,
  const ateam_geometry::Point & new_position,
  const ateam_geometry::Rectangle & defense_area)
{
  const auto area_center = CGAL::midpoint(defense_area.min(), defense_area.max());

  return CGAL::compare_distance_to_point(area_center, position, new_position) == CGAL::SMALLER;
}

}  // namespace ateam_kenobi::defense_area_enforcement
