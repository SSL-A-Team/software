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


#ifndef CORE__PATH_PLANNING__OBSTACLES_HPP_
#define CORE__PATH_PLANNING__OBSTACLES_HPP_

#include <vector>
#include <ateam_geometry/any_shape.hpp>
#include "core/types/world.hpp"

namespace ateam_kenobi::path_planning
{

std::vector<ateam_geometry::AnyShape> GetDefaultObstacles(const World & world);

void AddDefaultObstacles(const World & world, std::vector<ateam_geometry::AnyShape> & obstacles);

void AddRobotObstacles(
  const std::array<Robot, 16> & robots,
  const ateam_geometry::Point & ignore_point,
  std::vector<ateam_geometry::AnyShape> & obstacles);

void AddRobotObstacles(
  const std::array<Robot, 16> & robots, const int & ignore_id,
  std::vector<ateam_geometry::AnyShape> & obstacles);

void AddRobotObstacles(
  const std::array<Robot, 16> & robots,
  std::vector<ateam_geometry::AnyShape> & obstacles);

ateam_geometry::AnyShape MakeObstacleForRobot(const Robot & robot);

std::optional<ateam_geometry::AnyShape> GetCollidingObstacle(
  const ateam_geometry::AnyShape & footprint,
  const std::vector<ateam_geometry::AnyShape> & obstacles);

/**
 * @brief Checks if the given point is within the bounds of the field.
 *
 * @param apply_offset If true, offset the field bounds inward by approximately one robot radius
 */
bool IsPointInBounds(
  const ateam_geometry::Point & state, const World & world,
  const bool apply_offset = true);

}  // namespace ateam_kenobi::path_planning

#endif  // CORE__PATH_PLANNING__OBSTACLES_HPP_
