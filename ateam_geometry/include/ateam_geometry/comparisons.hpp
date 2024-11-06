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

#ifndef ATEAM_GEOMETRY__COMPARISONS_HPP_
#define ATEAM_GEOMETRY__COMPARISONS_HPP_

#include "types.hpp"
#include "normalize.hpp"
#include "epsilon.hpp"

namespace ateam_geometry
{

inline bool nearEqual(const Point & a, const Point & b, const double threshold = kDistanceEpsilon)
{
  return CGAL::squared_distance(a, b) < (threshold * threshold);
}

inline bool nearEqual(const Vector & a, const Vector & b, const double threshold = kDistanceEpsilon)
{
  return std::hypot(a.x() - b.x(), a.y() - b.y()) < threshold;
}

inline bool nearEqual(
  const Direction & a, const Direction & b,
  const double threshold = kDistanceEpsilon)
{
  return nearEqual(normalize(a.vector()), normalize(b.vector()), threshold);
}

}  // namespace ateam_geometry

#endif  // ATEAM_GEOMETRY__COMPARISONS_HPP_
