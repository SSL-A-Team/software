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
#include "ateam_geometry/rectangle.hpp"
#include <Eigen/Dense>
#include <math.h>

namespace ateam_geometry
{
Rectangle::Rectangle(const Eigen::Vector2d & v1, const Eigen::Vector2d & v2)
{
  width = std::abs(v1.x() - v1.y());
  height = std::abs(v1.y() - v2.y());

  corners[0] = v1;
  corners[1] = v2;
  corners[2] = Eigen::Vector2d(v1.x(), v2.y());
  corners[3] = Eigen::Vector2d(v2.x(), v1.y());

  area = width * height;
}

Eigen::Vector2d Rectangle::get_center()
{
  Eigen::Vector2d * least_positive = &corners[0];
  for (Eigen::Vector2d & corner : corners) {
    if (corner.x() <= least_positive->x() && corner.y() <= least_positive->y()) {
      least_positive = &corner;
    }
  }
  Eigen::Vector2d center;
  center.x() = least_positive->x() + (width / 2);
  center.y() = least_positive->y() + (height / 2);
  return center;
}

bool is_point_in_rectangle(Eigen::Vector2d & point, const Rectangle & rect)
{
  double max_x = rect.corners[0].x();
  double max_y = rect.corners[0].y();
  double min_x = rect.corners[0].x();
  double min_y = rect.corners[0].y();

  for (const Eigen::Vector2d & corner : rect.corners) {
    if (corner.x() > max_x) {
      max_x = corner.x();
    }
    if (corner.y() > max_y) {
      max_y = corner.y();
    }
    if (corner.x() < min_x) {
      min_x = corner.x();
    }
    if (corner.y() < min_y) {
      min_y = corner.y();
    }
  }
  if (point.x() <= max_x && point.x() >= min_x) {
    if (point.y() <= max_y && point.y() >= min_y) {
      return true;
    }
  }
  return false;
}
}  // namespace ateam_geometry
