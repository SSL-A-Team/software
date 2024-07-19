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

#include "obstacles.hpp"
#include <ranges>
#include <algorithm>
#include <ateam_geometry/creation_helpers.hpp>
#include <ateam_geometry/do_intersect.hpp>
#include <ateam_common/robot_constants.hpp>

namespace ateam_kenobi::path_planning
{

std::vector<ateam_geometry::AnyShape> GetDefaultObstacles(const World & world)
{
  std::vector<ateam_geometry::AnyShape> obstacles;
  AddDefaultObstacles(world, obstacles);
  return obstacles;
}

void AddDefaultObstacles(const World & world, std::vector<ateam_geometry::AnyShape> & obstacles)
{
  if (world.referee_info.running_command == ateam_common::GameCommand::BallPlacementOurs ||
    world.referee_info.running_command == ateam_common::GameCommand::BallPlacementTheirs)
  {
    return;
  }

  const auto half_field_length = world.field.field_length / 2.0;
  const auto half_defense_area_width = world.field.defense_area_width / 2.0;
  const auto back_x = half_field_length + ( 2 * world.field.boundary_width) +
    world.field.defense_area_depth;
  const auto defense_area_front_x = half_field_length - world.field.defense_area_depth;

  // Our defense area, extended to wall
  obstacles.push_back(
    ateam_geometry::Rectangle{
      ateam_geometry::Point{
        -back_x,
        -half_defense_area_width
      },
      ateam_geometry::Point{
        -defense_area_front_x,
        half_defense_area_width
      }
    }
  );

  // Their defense area, extended to wall
  obstacles.push_back(
    ateam_geometry::Rectangle{
      ateam_geometry::Point{
        back_x,
        -half_defense_area_width
      },
      ateam_geometry::Point{
        defense_area_front_x,
        half_defense_area_width
      }
    }
  );
}

void AddRobotObstacles(
  const std::array<Robot, 16> & robots,
  const ateam_geometry::Point & ignore_point,
  std::vector<ateam_geometry::AnyShape> & obstacles)
{
  auto not_ignored = [&ignore_point](const Robot & robot) {
      return (robot.pos - ignore_point).squared_length() > std::pow(kRobotRadius, 2);
    };

  auto robot_is_visible = [](const Robot & robot) {
      return robot.visible;
    };

  auto robot_obstacles = robots |
    std::views::filter(robot_is_visible) |
    std::views::filter(not_ignored) |
    std::views::transform(MakeObstacleForRobot);
  obstacles.insert(
    obstacles.end(),
    robot_obstacles.begin(),
    robot_obstacles.end());
}

void AddRobotObstacles(
  const std::array<Robot, 16> & robots, const int & ignore_id,
  std::vector<ateam_geometry::AnyShape> & obstacles)
{
  auto not_ignored = [&ignore_id](const Robot & robot) {
      return robot.id != ignore_id;
    };

  auto robot_is_visible = [](const Robot & robot) {
      return robot.visible;
    };

  auto robot_obstacles = robots |
    std::views::filter(robot_is_visible) |
    std::views::filter(not_ignored) |
    std::views::transform(MakeObstacleForRobot);
  obstacles.insert(
    obstacles.end(),
    robot_obstacles.begin(),
    robot_obstacles.end());
}

void AddRobotObstacles(
  const std::array<Robot, 16> & robots,
  std::vector<ateam_geometry::AnyShape> & obstacles)
{
  auto robot_is_visible = [](const Robot & robot) {
      return robot.visible;
    };

  auto robot_obstacles = robots |
    std::views::filter(robot_is_visible) |
    std::views::transform(MakeObstacleForRobot);
  obstacles.insert(
    obstacles.end(),
    robot_obstacles.begin(),
    robot_obstacles.end());
}

ateam_geometry::AnyShape MakeObstacleForRobot(const Robot & robot)
{
  return ateam_geometry::AnyShape(ateam_geometry::makeDisk(robot.pos, kRobotRadius));
}

std::optional<ateam_geometry::AnyShape> GetCollidingObstacle(
  const ateam_geometry::AnyShape & footprint,
  const std::vector<ateam_geometry::AnyShape> & obstacles)
{
  const auto found_iter = std::find_if(
    obstacles.begin(), obstacles.end(), [&footprint](const auto & obstacle) {
      return ateam_geometry::doIntersect(footprint, obstacle);
    });
  if (found_iter == obstacles.end()) {
    return std::nullopt;
  }
  return *found_iter;
}


bool IsPointInBounds(const ateam_geometry::Point & state, const World & world)
{
  const auto x_bound = (world.field.field_length / 2.0) + world.field.boundary_width - kRobotRadius;
  const auto y_bound = (world.field.field_width / 2.0) + world.field.boundary_width - kRobotRadius;
  ateam_geometry::Rectangle pathable_region(ateam_geometry::Point(-x_bound, -y_bound),
    ateam_geometry::Point(x_bound, y_bound));

  if (world.ignore_side > 0) {
    pathable_region = ateam_geometry::Rectangle(
      ateam_geometry::Point(-x_bound, -y_bound),
      ateam_geometry::Point(0, y_bound));
  } else if (world.ignore_side < 0) {
    pathable_region = ateam_geometry::Rectangle(
      ateam_geometry::Point(0, y_bound),
      ateam_geometry::Point(x_bound, y_bound));
  }

  return CGAL::do_intersect(state, pathable_region);
}

}  // namespace ateam_kenobi::path_planning
