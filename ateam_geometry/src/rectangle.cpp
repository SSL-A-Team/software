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
  width = std::abs(v1.x() - v2.x());
  height = std::abs(v1.y() - v2.y());

  // TODO(Christian): Ensure standard order of corners, with
  // min x/y at index 0.
  corners.at(0) = v1;
  corners.at(1) = v2;
  corners.at(2) = Eigen::Vector2d(v1.x(), v2.y());
  corners.at(3) = Eigen::Vector2d(v2.x(), v1.y());

  area = width * height;
}

Eigen::Vector2d Rectangle::get_center()
{
  Eigen::Vector2d center;
  double x_sum = 0;
  double y_sum = 0;
  for (auto & corner : corners) {
    x_sum += corner.x();
    y_sum += corner.y();
  }
  center.x() = x_sum / 4;
  center.y() = y_sum / 4;
  return center;
}

double Rectangle::get_area()
{
  return width * height;
}

std::array<Eigen::Vector2d, 4> Rectangle::get_corners()
{
  return corners;
}

bool is_point_in_rectangle(Eigen::Vector2d & point, Rectangle & rect)
{
  std::array<Eigen::Vector2d, 4> corners = rect.get_corners();
  double max_x = corners.at(0).x();
  double max_y = corners.at(0).y();
  double min_x = corners.at(0).x();
  double min_y = corners.at(0).y();

  for (const Eigen::Vector2d & corner : corners) {
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
