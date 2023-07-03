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

#ifndef TRAJECTORY_GENERATION__B_SPLINE_WRAPPER_HPP_
#define TRAJECTORY_GENERATION__B_SPLINE_WRAPPER_HPP_

#include <Eigen/Dense>

#include <vector>

#include "types/trajectory.hpp"

namespace BSplineWrapper
{

Trajectory Generate(
  const std::vector<Eigen::Vector2d> waypoints,
  const double start_heading,
  const double end_heading,
  const Eigen::Vector3d & start_vel,
  const Eigen::Vector3d & end_vel,
  const Eigen::Vector3d & max_vel_limits,
  const Eigen::Vector3d & max_accel_limits,
  const double dt, const double current_time);

}  // namespace BSplineWrapper

#endif  // TRAJECTORY_GENERATION__B_SPLINE_WRAPPER_HPP_
