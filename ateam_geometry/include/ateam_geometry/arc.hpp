// Copyright 2024 A Team
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

#ifndef ATEAM_GEOMETRY__ARC_HPP_
#define ATEAM_GEOMETRY__ARC_HPP_

#include <CGAL/Simple_cartesian.h>
#include "types.hpp"

namespace ateam_geometry
{

/**
 * @brief Represents a circular arc
 *
 * The arc represented runs counter-clockwise from start direction to end direction.
 */
class Arc
{
public:
  Arc() {}

  Arc(Point center, double radius, Direction start, Direction end)
  : center_(center), radius_(radius), start_(start), end_(end) {}

  const Point & center() const {return center_;}
  Point & center() {return center_;}

  const double & radius() const {return radius_;}
  double & radius() {return radius_;}

  const Direction & start() const {return start_;}
  Direction & start() {return start_;}

  const Direction & end() const {return end_;}
  Direction & end() {return end_;}

  Point source() const
  {
    return center_ + (radius_ * start_.vector());
  }

  Point target() const
  {
    return center_ + (radius_ * end_.vector());
  }

  double length() const
  {
    return angle() * radius_;
  }

  double angle() const
  {
    return std::abs(std::atan2(end_.dy(), end_.dx()) - std::atan2(start_.dy(), start_.dx()));
  }

private:
  Point center_;
  double radius_;
  Direction start_;
  Direction end_;
};

}  // namespace ateam_geometry

#endif  // ATEAM_GEOMETRY__ARC_HPP_
