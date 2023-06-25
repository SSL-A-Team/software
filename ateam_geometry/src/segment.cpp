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
#include <math.h>
#include <Eigen/Dense>

#include <optional>
#include <vector>

#include "ateam_geometry/segment.hpp"
#include "ateam_geometry/utilities.hpp"
#include "ateam_common/equality_utilities.hpp"

namespace ateam_geometry
{
LineSegment::LineSegment(const Eigen::Vector2d & p1, const Eigen::Vector2d & p2)
{
  this->p1 = p1;
  this->p2 = p2;
}

LineSegment::LineSegment(
  const Eigen::Vector2d & start, const double & length,
  const double & angle)
{
  p1 = start;
  Eigen::Vector2d endpoint;
  // Essentially we convert a given polar coordinate/vector
  // (length and angle) to an x, y plane
  endpoint.x() = start.x() + (length * cos(angle));
  endpoint.y() = start.y() + (length * sin(angle));
  p2 = endpoint;
}

double LineSegment::get_length()
{
  // Length is Euclidian, we can change later if desired
  return (p2 - p1).norm();
}

Eigen::Vector2d LineSegment::get_midpoint()
{
  return p1 + 0.5 * (p2 - p1);
}

std::vector<Eigen::Vector2d> LineSegment::get_equally_spaced_points(const int & num_points)
{
  /*
      Get equally spaced points along the line segement this is called from.
      If we only get one point, it will be the first endpoint.
      */
  std::vector<Eigen::Vector2d> points;
  if (num_points == 1) {
    points.push_back(this->p1);
    return points;
  }
  Eigen::Vector2d spacing;
  spacing.x() = (p2.x() - p1.x()) / (num_points - 1);
  spacing.y() = (p2.y() - p1.y()) / (num_points - 1);
  for (int i = 0; i < num_points; ++i) {
    Eigen::Vector2d point = this->p1;
    point = point + (spacing * i);
    points.push_back(point);
  }
  return points;
}

bool is_point_on_segment(const Eigen::Vector2d & point, LineSegment & segment, double tolerance)
{
  /*
      https://computergraphics.stackexchange.com/questions/2105/test-if-a-point-is-on-a-line-segment
      https://lucidar.me/en/mathematics/check-if-a-point-belongs-on-a-line-segment/
  */
  double crossProduct = (point.y() - segment.p1.y()) * (segment.p2.x() - segment.p1.x()) -
    (point.x() - segment.p1.x()) * (segment.p2.y() - segment.p1.y());

  if (std::abs(crossProduct) > tolerance) {
    return false;
  }

  double dotProduct = (point.x() - segment.p1.x()) * (segment.p2.x() - segment.p1.x()) +
    (point.y() - segment.p1.y()) * (segment.p2.y() - segment.p1.y());
  double segmentDotProduct = pow(segment.p2.x() - segment.p1.x(), 2) + pow(
    segment.p2.y() - segment.p1.y(), 2);

  if (dotProduct < -tolerance || dotProduct > segmentDotProduct + tolerance) {
    return false;
  }

  return true;
}

std::optional<Eigen::Vector2d> get_segment_intersection(
  const LineSegment & ls1,
  const LineSegment & ls2,
  double tolerance)
{
  /*
      Based on the algorithm implementation provided here:
      https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect

      There are two line segments, one running from p to p + r and another spanning
      q to q + s.

      Any point on this first line can be represented as p + tr (with scalar
      parameter t) and any point on the second can be represented as q + us
      (with scalar parameter u).

      The two lines intersect if we can find t and u such that:
      p + t r = q + u s

      We can solve for t:
      t = (q − p) × s / (r × s)

      And we can solve for u:
      u = (q − p) × r / (r × s)

      (See Stack Overflow for the algebra, I'm lazy.)

      Based on these definitions, there are 4 possible cases of
      whether or not 2 line segments intersect:

      1.If r x s = 0 && (q - p) x r = 0
          - The two segments are colinear
          - See the inline comments for this one, it's complicated
      2. If r x s = 0 && (q - p) x r != 0
          - Segments are parallel and non-intersecting
      3. If r x s != 0 && 0 <= t <= 1 && 0 <= u <= 1
          - Segments meet at p + tr = q + us
      4. Else
          - Segments are not parallel and do not intersect
      */
  Eigen::Vector2d p = ls1.p1;
  Eigen::Vector2d q = ls2.p1;
  Eigen::Vector2d r, s;

  r.x() = ls1.p2.x() - ls1.p1.x();
  r.y() = ls1.p2.y() - ls1.p1.y();

  s.x() = ls2.p2.x() - ls2.p1.x();
  s.y() = ls2.p2.y() - ls2.p1.y();

  double rxs = cross_product_2d(r, s);
  // Need to change this to be within epsilon?
  if (ateam_common::floatsClose(rxs, 0.0)) {
    if (ateam_common::floatsClose(cross_product_2d(q - p, r), 0.0)) {
      // The two segments are colinear
      // See if they intersect by getting the 2nd segments' two end points'
      // position relative to a projection on the 1st line
      // Which interval (either [t_0, t_1] or [t_1, t_0]) we need to check
      // depends on the two segments' relative directions.
      double s_dot_r = s.dot(r);
      // Check if the first end point is too far away
      double t_0 = (q - p).dot(r) / r.dot(r);
      double t_1 = t_0 + s_dot_r / r.dot(r);
      // Vectors point in the same direction
      if (s_dot_r > tolerance) {
        if (t_0 < tolerance) {
          // LS2 starts before LS1
          if (t_1 < tolerance) {
            // It does not interesct at all
            return std::nullopt;
          } else {
            // Return the start of LS1
            return p;
          }
        } else {
          // LS2 starts exactly on or after the first
          // endpoint of LS1
          if (t_1 > 1 + tolerance) {
            if (t_0 > 1 + tolerance) {
              return std::nullopt;
            }
          }
          // Return the starting point of LS2/LS1
          // (they are the same)
          return q;
        }
        // LS point in the opposite direction
      } else {
        // LS2 starts exactly on or after LS1 starts
        if (t_0 > 1 + tolerance) {
          if (t_1 > 1 + tolerance) {
            // LS2 is past LS1
            return std::nullopt;
          } else {
            // Return the ending point of LS1
            // since we are in opposite directions
            return ls1.p2;
          }
          // Return the starting point of LS2
          // Flipped because the LS are in opposite directions
        } else {
          return ls2.p2;
        }
      }
    } else {
      // Lines are parallel and non-intersecting
      return std::nullopt;
    }
  } else {
    double t = cross_product_2d(q - p, s) / rxs;
    double u = cross_product_2d(q - p, r) / rxs;
    if (0 <= t && t <= 1 && 0 <= u && u <= 1) {
      // The intersection is at p + tr = q + us
      Eigen::Vector2d intersection;
      intersection.x() = p.x() + (t * r.x());
      intersection.y() = p.y() + (t * r.y());
      return intersection;
    } else {
      // Segments are not parallel and do not intersect
      return std::nullopt;
    }
  }
}
}  // namespace ateam_geometry
