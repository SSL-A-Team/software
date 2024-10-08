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

#ifndef ATEAM_GEOMETRY__INTERSECTION_HPP_
#define ATEAM_GEOMETRY__INTERSECTION_HPP_

#include <CGAL/intersections.h>
#include <utility>
#include "types.hpp"
#include "arc.hpp"

namespace ateam_geometry
{

template<typename A, typename B>
typename CGAL::Intersection_traits<Kernel, A, B>::result_type intersection(const A & a, const B & b)
{
  return CGAL::intersection(a, b);
}

std::optional<std::variant<Point, std::pair<Point, Point>>> intersection(
  const Circle & circle,
  const Line & line);

std::optional<std::variant<Point, std::pair<Point, Point>>> intersection(
  const Arc & arc,
  const Line & line);

std::optional<std::variant<Point, std::pair<Point, Point>>> intersection(
  const Arc & arc,
  const Ray & ray);

std::optional<std::variant<Point, std::pair<Point, Point>>> intersection(
  const Arc & arc,
  const Segment & segment);

std::optional<std::variant<Point, Segment>> intersection(const Segment & a, const Segment & b);

}  // namespace ateam_geometry

#endif  // ATEAM_GEOMETRY__INTERSECTION_HPP_
