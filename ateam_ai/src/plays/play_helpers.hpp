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

#ifndef PLAYS__PLAY_HELPERS_HPP_
#define PLAYS__PLAY_HELPERS_HPP_

#include <algorithm>
#include <optional>

#include "ateam_geometry/ateam_geometry.hpp"
#include "ateam_geometry/eigen_conversions.hpp"
#include "ateam_common/robot_constants.hpp"
#include "types/field.hpp"
#include "types/world.hpp"

bool is_point_in_bounds(ateam_geometry::Point & point, Field & field)
{
  if (point.x() > field.field_width / 2 || point.x() < -field.field_width / 2) {
    return false;
  }
  if (point.y() > field.field_length / 2 || point.y() < -field.field_length / 2) {
    return false;
  }
  return true;
}

bool is_point_in_defense_area(ateam_geometry::Point & point, Field & field){};

bool is_point_in_robot(ateam_geometry::Point & candidate_point, World & world){
  auto check_point = [&candidate_point](const std::optional<Robot> robot){
    return robot.has_value() && (position - robot->pos).norm() <= kRobotDiameter;
  }
  
  return std::ranges::any_of(world.our_robots, check_point) || std::ranges::any_of(world.their_robots, check_point);
};

#endif  // PLAYS__PLAY_HELPERS_HPP_
