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

#ifndef ATEAM_GEOMETRY__SEGMENT_HPP_
#define ATEAM_GEOMETRY__SEGMENT_HPP_

#include <Eigen/Dense>

#include <vector>
#include <optional>

namespace ateam_geometry
{
class LineSegment
{
public:
  LineSegment(const Eigen::Vector2d & p1, const Eigen::Vector2d & p2);
  LineSegment(const Eigen::Vector2d & start, const double & length, const double & angle);

  double get_length();
  Eigen::Vector2d get_midpoint();
  std::vector<Eigen::Vector2d> get_equally_spaced_points(const int & num_points);

  Eigen::Vector2d p1;
  Eigen::Vector2d p2;
};

bool is_point_on_segment(
  const Eigen::Vector2d & point, LineSegment & segment,
  double tolerance = 1e-6);

/*Given two 2d line segments, return a < std::optional<Eigen::Vector2d>
corresponding to whether or not they intersect and if applicable,
the point of intersection.*/
std::optional<Eigen::Vector2d> get_segment_intersection(
  const LineSegment & ls1,
  const LineSegment & ls2,
  double tolerance = 1e-6);
}  // namespace ateam_geometry

#endif   // ATEAM_GEOMETRY__SEGMENT_HPP_
