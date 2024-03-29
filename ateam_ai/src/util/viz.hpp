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

#ifndef UTIL__VIZ_HPP_
#define UTIL__VIZ_HPP_

#include <Eigen/Dense>
#include <vector>

#include <ateam_common/overlay.hpp>

#include "types/trajectory.hpp"

namespace viz
{
void DrawTrajectory(const int robot_id, const Trajectory & trajectory)
{
  if (trajectory.samples.size() <= 1) {
    return;
  }

  std::vector<Eigen::Vector2d> line_pts;
  for (const auto & sample : trajectory.samples) {
    line_pts.push_back(Eigen::Vector2d{sample.pose.x(), sample.pose.y()});
  }

  ateam_common::Overlay::GetOverlay().DrawLine(line_pts, "trajectory_" + std::to_string(robot_id));
}

}  // namespace viz

#endif  // UTIL__VIZ_HPP_
