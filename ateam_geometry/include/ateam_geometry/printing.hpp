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

#ifndef ATEAM_GEOMETRY__PRINTING_HPP_
#define ATEAM_GEOMETRY__PRINTING_HPP_

#include <algorithm>
#include <iostream>
#include "types.hpp"
#include "disk.hpp"
#include "arc.hpp"
#include "any_shape.hpp"

namespace ateam_geometry
{

inline std::ostream & operator<<(std::ostream & os, const ateam_geometry::Point & p)
{
  os << "Point{x:" << p.x() << ", y:" << p.y() << '}';
  return os;
}

inline std::ostream & operator<<(std::ostream & os, const ateam_geometry::Segment & s)
{
  os << "Segment{s:" << s.source() << ", t:" << s.target() << '}';
  return os;
}

inline std::ostream & operator<<(std::ostream & os, const ateam_geometry::Ray & r)
{
  os << "Ray{src:" << r.source() << ", dir:" << r.direction() << '}';
  return os;
}

inline std::ostream & operator<<(std::ostream & os, const ateam_geometry::Rectangle & r)
{
  const ateam_geometry::Point min_point{r.xmin(), r.ymin()};
  const ateam_geometry::Point max_point{r.xmax(), r.ymax()};
  os << "Rectangle{min:" << min_point << ", max:" << max_point << '}';
  return os;
}

inline std::ostream & operator<<(std::ostream & os, const ateam_geometry::Circle & c)
{
  os << "Circle{center:" << c.center() << ", radius:" <<
    CGAL::approximate_sqrt(c.squared_radius()) << '}';
  return os;
}

inline std::ostream & operator<<(std::ostream & os, const ateam_geometry::Line & l)
{
  os << "Line{point:" << l.point() << ", direction:" << l.direction() << '}';
  return os;
}

inline std::ostream & operator<<(std::ostream & os, const ateam_geometry::Polygon & p)
{
  os << "Polygon{";
  std::copy(p.begin(), p.end(), std::ostream_iterator<ateam_geometry::Point>(os, ", "));
  os << '}';
  return os;
}

inline std::ostream & operator<<(std::ostream & os, const ateam_geometry::Vector & v)
{
  os << "Vector{x:" << v.x() << ", y:" << v.y() << '}';
  return os;
}

inline std::ostream & operator<<(std::ostream & os, const ateam_geometry::Direction & d)
{
  os << "Direction{dx:" << d.dx() << ", dy:" << d.dy() << '}';
  return os;
}

inline std::ostream & operator<<(std::ostream & os, const ateam_geometry::Arc & a)
{
  os << "Arc{center:" << a.center() << ", radius:" << a.radius() << ", start:" << a.start() <<
    ", end:" << a.end() << '}';
  return os;
}

inline std::ostream & operator<<(std::ostream & os, const ateam_geometry::Disk & d)
{
  os << "Disk{center:" << d.center() << ", radius:" << CGAL::approximate_sqrt(d.squared_radius()) <<
    '}';
  return os;
}

inline std::ostream & operator<<(std::ostream & os, const ateam_geometry::AnyShape & a)
{
  std::visit([&os](const auto & s){
      os << s;
  }, a);
  return os;
}

}  // namespace ateam_geometry

#endif  // ATEAM_GEOMETRY__PRINTING_HPP_
