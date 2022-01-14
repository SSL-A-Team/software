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

#include "generators/generator_util.hpp"

#include <limits>

#include "types/models.hpp"

std::optional<Robot> generator_util::get_closest_robot(const Eigen::Vector2d & position, 
    const std::array<std::optional<Robot>, 16> & blue_robots,
    const std::array<std::optional<Robot>, 16> & yellow_robots)
{
  std::optional<Robot> closest_robot = std::nullopt;
  double dist = std::numeric_limits<double>::infinity();

  for (const auto & robot : blue_robots) {
    if (robot.has_value()) {
      Eigen::Vector2d robot_pos = robot.value().position;

      double test_dist = (position - robot_pos).norm();
      if (test_dist < dist) {
        closest_robot = robot;
        dist = test_dist;
      }
    }
  }

  for (const auto & robot : yellow_robots) {
    if (robot.has_value()) {
      Eigen::Vector2d robot_pos = robot.value().position;

      double test_dist = (position - robot_pos).norm();
      if (test_dist < dist) {
        closest_robot = robot;
        dist = test_dist;
      }
    }
  }

  return closest_robot;
}


bool generator_util::is_near_robot(
  const Eigen::Vector2d & position,
  const std::optional<Robot> & robot)
{
  if (!robot.has_value()) {
    return false;
  }

  return (robot.value().position - position).norm() < 0.5;
}

bool generator_util::is_in_robot_mouth(
  const Eigen::Vector2d & position,
  const std::optional<Robot> & robot)
{
  if (!robot.has_value()) {
    return false;
  }

  // https://en.wikipedia.org/wiki/Vector_projection
  Eigen::Vector2d heading{cos(robot.value().theta), sin(robot.value().theta)};
  Eigen::Vector2d robot_to_ball = position - robot.value().position;

  return cos(Models::Robot::mouth_half_angle) <
         heading.dot(robot_to_ball) / (heading.norm() * robot_to_ball.norm());
}

bool generator_util::is_moving_towards_robot(
  const Eigen::Vector2d & position,
  const Eigen::Vector2d & velocity,
  const std::optional<Robot> & robot)
{
  Eigen::Vector2d ball_to_robot = robot.value().position - position;

  return ball_to_robot.dot(velocity) > 0;
}
