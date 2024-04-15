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

class Arc
{
public:
  Arc() {}

  Arc(Point center, double radius, double start, double end)
  : center_(center), radius_(radius), start_angle_(start), end_angle_(end) {}

  const Point & center() const {return center_;}
  Point & center() {return center_;}

  const double & radius() const {return radius_;}
  double & radius() {return radius_;}

  const double & start() const {return start_angle_;}
  double & start() {return start_angle_;}

  const double & end() const {return end_angle_;}
  double & end() {return end_angle_;}

  Point source() const
  {
    return center_ + (radius_ * Vector(std::cos(start_angle_), std::sin(start_angle_)));
  }

  Point target() const
  {
    return center_ + (radius_ * Vector(std::cos(end_angle_), std::sin(end_angle_)));
  }

  double length() const
  {
    return std::abs(end_angle_ - start_angle_) * radius_;
  }

  double angle() const
  {
    return std::abs(end_angle_ - start_angle_);
  }

private:
  Point center_;
  double radius_;
  double start_angle_;
  double end_angle_;
};

}  // namespace ateam_geometry

#endif  // ATEAM_GEOMETRY__ARC_HPP_
