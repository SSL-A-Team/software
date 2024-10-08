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

#ifndef ATEAM_GEOMETRY__ORIENTATION_HPP_
#define ATEAM_GEOMETRY__ORIENTATION_HPP_

#include "ateam_geometry/types.hpp"
#include "ateam_geometry/epsilon.hpp"


namespace ateam_geometry
{

enum class Orientation
{
  Colinear,
  Clockwise,
  Counterclockwise
};

/**
 * @brief Determines the orientation of the points in order A->B->C
 * @note Based on https://www.geeksforgeeks.org/orientation-3-ordered-points/
 */
inline Orientation orientation(const Point & a, const Point & b, const Point & c)
{
  const auto slope_diff = (b.y() - a.y()) * (c.x() - b.x()) - (b.x() - a.x()) * (c.y() - b.y());
  if (std::abs(slope_diff) < kGenericEpsilon) {
    return Orientation::Colinear;
  }
  return slope_diff > 0 ? Orientation::Clockwise : Orientation::Counterclockwise;
}

}  // namespace ateam_geometry

#endif  // ATEAM_GEOMETRY__ORIENTATION_HPP_
