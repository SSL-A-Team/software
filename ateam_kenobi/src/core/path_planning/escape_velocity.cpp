// Copyright 2025 A Team
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

#include "escape_velocity.hpp"
#include <algorithm>
#include <ateam_common/robot_constants.hpp>
#include "obstacles.hpp"

namespace ateam_kenobi::path_planning
{

ateam_geometry::Vector GenerateEscapeVelocity(
  const Robot & robot,
  const ateam_geometry::AnyShape & obstacle)
{
  if (std::holds_alternative<ateam_geometry::Circle>(obstacle)) {
    const auto & circle = std::get<ateam_geometry::Circle>(obstacle);
    return GenerateEscapeVelocity(robot, circle);
  }

  if (std::holds_alternative<ateam_geometry::Disk>(obstacle)) {
    const auto & disk = std::get<ateam_geometry::Disk>(obstacle);
    return GenerateEscapeVelocity(robot, disk);
  }

  if (std::holds_alternative<ateam_geometry::Rectangle>(obstacle)) {
    const auto & rectangle = std::get<ateam_geometry::Rectangle>(obstacle);
    return GenerateEscapeVelocity(robot, rectangle);
  }

  // Default case when we don't recognize the obstacle type
  return ateam_geometry::Vector{};
}

ateam_geometry::Vector GenerateEscapeVelocity(
  const Robot & robot,
  const ateam_geometry::Circle & obstacle)
{
  const auto vector = robot.pos - obstacle.center();
  return ateam_geometry::normalize(vector) * kSafeEscapeVelocity;
}

ateam_geometry::Vector GenerateEscapeVelocity(
  const Robot & robot,
  const ateam_geometry::Disk & obstacle)
{
  const auto vector = robot.pos - obstacle.center();
  return ateam_geometry::normalize(vector) * kSafeEscapeVelocity;
}

ateam_geometry::Vector GenerateEscapeVelocity(
  const Robot & robot,
  const ateam_geometry::Rectangle & obstacle)
{
  std::array<ateam_geometry::Segment, 4> edges = {
    ateam_geometry::Segment{obstacle[0], obstacle[1]},
    ateam_geometry::Segment{obstacle[1], obstacle[2]},
    ateam_geometry::Segment{obstacle[2], obstacle[3]},
    ateam_geometry::Segment{obstacle[3], obstacle[0]},
  };
  std::array<double, 4> distances;
  std::ranges::transform(
        edges, distances.begin(), [&robot](const auto & edge) {
      return CGAL::squared_distance(robot.pos, edge);
        });
  const auto min_iter = std::ranges::min_element(distances);

  const auto closest_edge = edges[std::distance(distances.begin(), min_iter)];

  const auto edge_direction = closest_edge.direction();

  // Rotate direction by -90 deg. Assumes vertices are reported in counterclockwise order
  const ateam_geometry::Vector escape_vector{edge_direction.dy(), -edge_direction.dx()};

  return escape_vector * kSafeEscapeVelocity;
}

std::optional<ateam_geometry::Vector> GenerateEscapeVelocity(
  const Robot & robot,
  const std::vector<ateam_geometry::AnyShape> & obstacles, double footprint_inflation)
{
  const auto robot_footprint = ateam_geometry::makeDisk(robot.pos,
      kRobotRadius + footprint_inflation);

  /* TODO(barulicm): Using only the first colliding obstacle to generate escape velocities may not
   * yield a useful velocity when multiple collisions are occurring simultaneously.
   */
  const auto colliding_obstacle = path_planning::GetCollidingObstacle(robot_footprint, obstacles);
  if (!colliding_obstacle) {
    return std::nullopt;
  }
  return GenerateEscapeVelocity(robot, *colliding_obstacle);
}

}  // namespace ateam_kenobi::path_planning
