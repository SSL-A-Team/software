// Copyright 2026 A Team
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

#ifndef ATEAM_PATH_PLANNING__OBSTACLE_HPP_
#define ATEAM_PATH_PLANNING__OBSTACLE_HPP_

#include <variant>
#include <vector>
#include <ateam_common/robot_constants.hpp>
#include <ateam_game_state/robot.hpp>
#include <ateam_geometry/any_shape.hpp>
#include <ateam_geometry/creation_helpers.hpp>
#include <ateam_geometry/types.hpp>

namespace ateam_path_planning
{

struct ObstacleTrajectory
{
  std::vector<ateam_geometry::Point> points;
  double time_step;
};

struct Obstacle
{
  ateam_geometry::AnyShape shape;
  std::variant<std::monostate, ateam_geometry::Vector, ObstacleTrajectory> expected_motion;

  static Obstacle FromRobot(const ateam_game_state::Robot & robot)
  {
    return Obstacle{ateam_geometry::makeDisk(robot.pos, kRobotRadius), robot.vel};
  }

  static Obstacle FromRobot(
    const ateam_game_state::Robot & robot,
    const std::vector<ateam_geometry::Point> & trajectory, double time_step)
  {
    return Obstacle{ateam_geometry::makeDisk(robot.pos, kRobotRadius),
      ObstacleTrajectory{trajectory, time_step}};
  }

  /// Transform the obstacle shape t seconds into the future based on @c expected_motion
  ateam_geometry::AnyShape ShapeAtT(const double t) const;
};

}  // namespace ateam_path_planning

#endif  // ATEAM_PATH_PLANNING__OBSTACLE_HPP_
