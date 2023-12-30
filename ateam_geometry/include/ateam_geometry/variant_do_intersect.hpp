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

#ifndef ATEAM_GEOMETRY__VARIANT_DO_INTERSECT_HPP_
#define ATEAM_GEOMETRY__VARIANT_DO_INTERSECT_HPP_

#include <variant>
#include "ateam_geometry/types.hpp"
#include "ateam_geometry/disk_intersection.hpp"

namespace ateam_geometry
{

/**
 * @brief Check if two geometry objects intersect.
 *
 * Intersection semantics are the same as @c CGAL::do_intersect
 *
 * @note When not using AnyShape, use @c CGAL::do_intersect
 *
 * @tparam ObjA Type of the non-variant geometry object
 * @param object_a A geometry object
 * @param object_b A variant holding a geometry object
 * @return true The two objects intersect
 * @return false The two objects do not intersect
 */
template<typename ObjA>
bool variantDoIntersect(const ObjA & object_a, const AnyShape & object_b)
{
  if constexpr (std::is_same_v<ObjA, Circle>) {
    return std::visit(
      [&object_a](const auto & b) {
        return ateam_geometry::doDiskIntersect(object_a, b);
      }, object_b);
  }
  if (std::holds_alternative<Circle>(object_b)) {
    return ateam_geometry::doDiskIntersect(std::get<Circle>(object_b), object_a);
  }
  return std::visit(
    [&object_a](const auto & b) {
      return CGAL::do_intersect(object_a, b);
    }, object_b);
}

/**
 * @brief Check if two AnyShape variants intersect.
 *
 * Intersection semantics are the same as @c CGAL::do_intersect
 *
 * @note When not using AnyShape, use @c CGAL::do_intersect
 *
 * @param object_a A variant holding a geometry object
 * @param object_b A variant holding a geometry object
 * @return true The two geometry objects intersect
 * @return false The two geometry objects do not intersect
 */
template<>
inline bool variantDoIntersect(const AnyShape & object_a, const AnyShape & object_b)
{
  return std::visit(
    [&object_b](const auto & a) {
      return variantDoIntersect(a, object_b);
    }, object_a);
}
}  // namespace ateam_geometry

#endif  // ATEAM_GEOMETRY__VARIANT_DO_INTERSECT_HPP_
