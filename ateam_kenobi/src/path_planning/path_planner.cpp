// Copyright 2021 A Team
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


#include "path_planner.hpp"
#include <ranges>
#include <algorithm>
#include <vector>
#include <ateam_common/robot_constants.hpp>
#include <ateam_geometry/ateam_geometry.hpp>

namespace ateam_kenobi::path_planning
{

PathPlanner::PathPlanner()
{
}

PathPlanner::Path PathPlanner::getPath(
  const Position & start, const Position & goal, const World & world,
  const std::vector<ateam_geometry::AnyShape> & obstacles,
  const PlannerOptions & options)
{
  const auto start_time = std::chrono::steady_clock::now();

  std::vector<ateam_geometry::AnyShape> augmented_obstacles = obstacles;

  addRobotsToObstacles(world, start, augmented_obstacles);

  if (options.use_default_obstacles) {
    addDefaultObstacles(world, augmented_obstacles);
  }

  if (options.avoid_ball) {
    augmented_obstacles.push_back(ateam_geometry::makeCircle(world.ball.pos, 0.04267 / 2));
  }

  if (!isStateValid(goal, world, augmented_obstacles, options)) {
    return {};
  }

  Path path = {start, goal};

  while (true) {
    const auto elapsed_time = std::chrono::duration_cast<std::chrono::duration<double>>(
      std::chrono::steady_clock::now() - start_time).count();
    if (elapsed_time > options.search_time_limit) {
      return path;
    }
    bool had_to_split = false;
    for (auto ind = 0u; ind < (path.size() - 1); ++ind) {
      auto split_result = splitSegmentIfNecessary(
        path, ind, ind + 1, world, augmented_obstacles,
        options);
      if (split_result.split_needed) {
        had_to_split = true;
        if (!split_result.split_succeeded) {
          // unable to find path around obstacle
          return {};
        }
        break;
      }
    }
    if (!had_to_split) {
      // no split -> path is collision free
      break;
    }
  }

  return path;
}

bool PathPlanner::isStateValid(
  const ateam_geometry::Point & state,
  const World & world,
  const std::vector<ateam_geometry::AnyShape> & obstacles,
  const PlannerOptions & options)
{
  const auto x_bound = (world.field.field_length / 2.0) + world.field.boundary_width - kRobotRadius;
  const auto y_bound = (world.field.field_width / 2.0) + world.field.boundary_width - kRobotRadius;
  const ateam_geometry::Rectangle pathable_region(
    ateam_geometry::Point(-x_bound, -y_bound),
    ateam_geometry::Point(x_bound, y_bound));
  if (!CGAL::do_intersect(state, pathable_region)) {
    return false;
  }

  auto robot_footprint = ateam_geometry::makeCircle(
    state,
    kRobotRadius + options.footprint_inflation
  );

  return std::ranges::none_of(
    obstacles, [&robot_footprint](const ateam_geometry::AnyShape & obstacle) {
      return ateam_geometry::variantDoIntersect(robot_footprint, obstacle);
    });
}

std::optional<ateam_geometry::Point> PathPlanner::getCollisionPoint(
  const ateam_geometry::Point & p1, const ateam_geometry::Point & p2, const World & world,
  const std::vector<ateam_geometry::AnyShape> & obstacles, const PlannerOptions & options)
{
  const auto direction_vector = p2 - p1;
  const auto segment_length = std::sqrt(direction_vector.squared_length());
  const auto step_vector = direction_vector * (options.collision_check_resolution / segment_length);
  const int step_count = segment_length / options.collision_check_resolution;
  for (auto step = 0; step < step_count; ++step) {
    const auto state = p1 + (step * step_vector);
    if (!isStateValid(state, world, obstacles, options)) {
      return state;
    }
  }
  if (!isStateValid(p2, world, obstacles, options)) {
    return p2;
  }
  return std::nullopt;
}

PathPlanner::SplitResult PathPlanner::splitSegmentIfNecessary(
  Path & path, const std::size_t ind1,
  const std::size_t ind2, const World & world,
  const std::vector<ateam_geometry::AnyShape> & obstacles,
  const PlannerOptions & options)
{
  const auto maybe_collision_point = getCollisionPoint(
    path.at(ind1), path.at(
      ind2), world, obstacles, options);
  if (!maybe_collision_point) {
    return {
      .split_needed = false,
      .split_succeeded = true
    };
  }
  const auto collision_point = maybe_collision_point.value();
  const auto p1 = path[ind1];
  const auto p2 = path[ind2];
  auto change_vector = (ateam_geometry::normalize(p2 - p1) * kRobotRadius).perpendicular(
    CGAL::CLOCKWISE);
  auto new_point = collision_point;
  auto direction = 1;
  for (auto i = 1; i < 100; ++i) {
    new_point = new_point + (change_vector * i * direction);
    direction *= -1;
    if (isStateValid(new_point, world, obstacles, options)) {
      path.insert(path.begin() + ind2, new_point);
      return {
        .split_needed = true,
        .split_succeeded = true
      };
    }
  }
  return {
    .split_needed = true,
    .split_succeeded = false
  };
}

void PathPlanner::addRobotsToObstacles(
  const World & world,
  const ateam_geometry::Point & start_pos,
  std::vector<ateam_geometry::AnyShape> & obstacles)
{

  auto obstacle_from_robot = [](const Robot & robot) {
      return ateam_geometry::AnyShape(
        ateam_geometry::makeCircle(robot.pos, kRobotRadius));
    };

  auto not_current_robot = [&start_pos](const Robot & robot) {
      // Assume any robot close enough to the start pos is the robot trying to navigate
      return (robot.pos - start_pos).squared_length() > std::pow(kRobotRadius, 2);
    };

  auto robot_is_visible = [](const Robot & robot) {
      return robot.visible;
    };

  auto our_robot_obstacles = world.our_robots |
    std::views::filter(robot_is_visible) |
    std::views::filter(not_current_robot) |
    std::views::transform(obstacle_from_robot);
  obstacles.insert(
    obstacles.end(),
    our_robot_obstacles.begin(),
    our_robot_obstacles.end());

  auto their_robot_obstacles = world.their_robots |
    std::views::filter(robot_is_visible) |
    std::views::transform(obstacle_from_robot);
  obstacles.insert(
    obstacles.end(),
    their_robot_obstacles.begin(),
    their_robot_obstacles.end());
}

void PathPlanner::addDefaultObstacles(
  const World & world,
  std::vector<ateam_geometry::AnyShape> & obstacles)
{
  // our goalie box
  obstacles.push_back(
    ateam_geometry::Rectangle(
      ateam_geometry::Point(-world.field.field_length / 2, world.field.goal_width),
      ateam_geometry::Point(
        -1 * (world.field.field_length / 2) + world.field.goal_depth,
        -world.field.goal_width)
  ));
  // their goalie box
  obstacles.push_back(
    ateam_geometry::Rectangle(
      ateam_geometry::Point((world.field.field_length / 2), world.field.goal_width),
      ateam_geometry::Point(
        (world.field.field_length / 2) - world.field.goal_depth,
        -world.field.goal_width)
  ));
}

}  // namespace ateam_kenobi::path_planning
