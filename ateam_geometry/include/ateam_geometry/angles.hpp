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

#ifndef ATEAM_GEOMETRY__ANGLES_HPP_
#define ATEAM_GEOMETRY__ANGLES_HPP_

#include <angles/angles.h>
#include "types.hpp"

namespace ateam_geometry
{

inline double ToHeading(const ateam_geometry::Direction & direction)
{
  return std::atan2(direction.dy(), direction.dx());
}

inline double ToHeading(const ateam_geometry::Vector & vector)
{
  return ToHeading(vector.direction());
}

inline double ShortestAngleBetween(
  const ateam_geometry::Direction & source,
  const ateam_geometry::Direction & target)
{
  return angles::shortest_angular_distance(ToHeading(source), ToHeading(target));
}

inline double ShortestAngleBetween(
  const ateam_geometry::Vector & source,
  const ateam_geometry::Vector & target)
{
  return ShortestAngleBetween(source.direction(), target.direction());
}

/**
 * @brief Checks if the angle @c a is in the clockwise range from @c start to @c end
 *
 * @param a the angle, in radians, to check
 * @param start the angle, in radians, defining the start of the range
 * @param end the angle, in radians, defining the end of the range
 * @return true @a is between @c start and @c end
 * @return false @a is outside of @c start and @c end
 */
inline bool IsClockwiseBetween(const double a, const double start, const double end)
{
  if(start > end) {
    return (a <= start && a <= end) || (a >= start && a >= end);
  } else {
    return (a >= start) && (a <= end);
  }
}

/**
 * @brief Checks if the angle @c a is in the counterclockwise range from @c start to @c end
 *
 * @param a the angle, in radians, to check
 * @param start the angle, in radians, defining the start of the range
 * @param end the angle, in radians, defining the end of the range
 * @return true @a is between @c start and @c end
 */
inline bool IsCounterclockwiseBetween(const double a, const double start, const double end)
{
  if(start < end) {
    return (a <= start && a <= end) || (a >= start && a >= end);
  } else {
    return (a <= start) && (a >= end);
  }
}

}  // namespace ateam_geometry

#endif  // ATEAM_GEOMETRY__ANGLES_HPP_
