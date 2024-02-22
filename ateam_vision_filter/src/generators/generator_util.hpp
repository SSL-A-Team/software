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

#ifndef GENERATORS__GENERATOR_UTIL_HPP_
#define GENERATORS__GENERATOR_UTIL_HPP_

#include <Eigen/Dense>

#include <array>
#include <optional>

#include "types/robot.hpp"


namespace ateam_vision_filter::generator_util
{
/**
 * @return Closest robot to the position given (if one exists)
 */
std::optional<Robot> get_closest_robot(
  const Eigen::Vector2d & position,
  const std::array<std::optional<Robot>, 16> & blue_robots,
  const std::array<std::optional<Robot>, 16> & yellow_robots);

/**
 * @return true if position is near the given robot
 */
bool is_near_robot(const Eigen::Vector2d & position, const std::optional<Robot> & robot);

/**
 * @return true if position within the mouth angle (not accounting for distance to mouth)
 */
bool is_in_robot_mouth(
  const Eigen::Vector2d & position,
  const std::optional<Robot> & robot);

/**
 * @return true if ball is moving +- 90 degrees of vector from current position to robot
 * @note Noise on the velocity vector at low speeds may produce inconsitent results
 */
bool is_moving_towards_robot(
  const Eigen::Vector2d & position, const Eigen::Vector2d & velocity,
  const std::optional<Robot> & robot);
}  // namespace generator_util

#endif  // GENERATORS__GENERATOR_UTIL_HPP_
