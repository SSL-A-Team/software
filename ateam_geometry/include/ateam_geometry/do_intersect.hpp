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

#ifndef ATEAM_GEOMETRY__DO_INTERSECT_HPP_
#define ATEAM_GEOMETRY__DO_INTERSECT_HPP_

#include <CGAL/intersection_2.h>
#include "any_shape.hpp"
#include "disk.hpp"
#include "types.hpp"

namespace ateam_geometry
{

/**
 * @brief Checks if two shapes intersect.
 *
 * Provided as an extension point for adding support for our custom types to
 * CGAL's do_intersect function.
 */
template<typename A, typename B>
bool doIntersect(const A & a, const B & b)
{
  return CGAL::do_intersect(a, b);
}

template<typename ObjA>
bool doIntersect(const ObjA & a, const AnyShape & b)
{
  return std::visit(
    [&a](const auto & any_shape_val) {
      return ateam_geometry::doIntersect(a, any_shape_val);
    }, b);
}

template<>
bool doIntersect(const AnyShape & a, const AnyShape & b);

template<>
bool doIntersect(const Disk & disk_a, const Disk & disk_b);

template<>
bool doIntersect(const Disk & disk, const Rectangle & rec);

template<>
bool doIntersect(const Disk & disk, const Point & point);

template<>
bool doIntersect(const Disk & disk, const Segment & segment);

template<>
bool doIntersect(const Disk & disk, const Ray & ray);

}  // namespace ateam_geometry

#endif  // ATEAM_GEOMETRY__DO_INTERSECT_HPP_
