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

#ifndef PATH_PLANNING__ESCAPE_VELOCITY_HPP_
#define PATH_PLANNING__ESCAPE_VELOCITY_HPP_

#include <vector>
#include <ateam_geometry/types.hpp>
#include <ateam_geometry/any_shape.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "types/robot.hpp"

namespace ateam_kenobi::path_planning
{

constexpr double kSafeEscapeVelocity = 0.5;  // m/s

/**
 * @brief Creates a low velocity in the quickest direction to escape the given obstacle.
 *
 * @note This function does not actually check if the robot is currently in the given obstacle.
 */
geometry_msgs::msg::Twist GenerateEscapeVelocity(
  const Robot & robot,
  const ateam_geometry::AnyShape & obstacle);

geometry_msgs::msg::Twist GenerateEscapeVelocity(
  const Robot & robot,
  const ateam_geometry::Circle & obstacle);

geometry_msgs::msg::Twist GenerateEscapeVelocity(
  const Robot & robot,
  const ateam_geometry::Disk & obstacle);

geometry_msgs::msg::Twist GenerateEscapeVelocity(
  const Robot & robot,
  const ateam_geometry::Rectangle & obstacle);

/**
 * @brief Creates a low velocity in the quickest direction to escape the first colliding obstacle
 * in @c obstacles.
 *
 * @return std::optional<geometry_msgs::msg::Twist> The escape velocity, or @c nullopt if no
 * obstacles collide with the robot.
 */
std::optional<geometry_msgs::msg::Twist> GenerateEscapeVelocity(
  const Robot & robot,
  const std::vector<ateam_geometry::AnyShape> & obstacles, double footprint_inflation = 0.06);

}  // namespace ateam_kenobi::path_planning

#endif  // PATH_PLANNING__ESCAPE_VELOCITY_HPP_
