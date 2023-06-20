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
#define _USE_MATH_DEFINES

#include <Eigen/Dense>
#include <math.h>
#include "ateam_geometry/circle.hpp"
#include "ateam_geometry/segment.hpp"

namespace ateam_geometry
{
Circle::Circle(const Eigen::Vector2d & c, const double & r)
{
  center = c;
  radius = r;
}

std::vector<Eigen::Vector2d> Circle::get_equally_spaced_points(
  const int & num_points,
  const double & offset = 0.0)
{
  std::vector<Eigen::Vector2d> points;
  // Assume we want to return more than 1 point...
  // If this is not the case, don't do any more calculations and return
  // an empty vector
  if (num_points < 2) {
    return points;
  }
  double spacing = (2 * M_PI) / num_points;
  for (int i = 0; i < num_points; ++i) {
    Eigen::Vector2d point =
      get_lineseg_of_length_from_point(center, radius, offset + (i * spacing)).p2;
    points.push_back(point);
  }
  return points;
}

bool is_point_in_circle(const Eigen::Vector2d & point, Circle & circle)
{
  Eigen::Vector2d line_to_center = Eigen::Vector2d(
    point.x() - circle.center.x(), point.y() - circle.center.y()
  );
  if (line_to_center.norm() <= circle.radius) {
    return true;
  } else {
    return false;
  }
}
}
