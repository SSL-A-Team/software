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

#ifndef ATEAM_GEOMETRY__CREATION_HELPERS_HPP_
#define ATEAM_GEOMETRY__CREATION_HELPERS_HPP_

#include "types.hpp"
#include "disk.hpp"

namespace ateam_geometry
{

/**
 * @brief Factory utility to work around CGAL wanting the squared radius in the circle constructor.
 *
 * @param center Center point of the circle
 * @param radius Radius of the circle
 * @return Circle
 */
inline Circle makeCircle(Point center, double radius)
{
  return ateam_geometry::Circle(center, radius * radius);
}

/**
 * @brief Factory utility to work around CGAL wanting the squared radius in the circle constructor
 * inheritted by Disk.
 *
 * @param center
 * @param radius
 * @return Disk
 */
inline Disk makeDisk(Point center, double radius)
{
  return Disk(center, radius * radius);
}

/**
 * @brief Creates a Direction from an angle
 *
 * @param angle Angle of the Direction in radians
 * @return Direction
 */
inline Direction directionFromAngle(double angle)
{
  return Direction(std::cos(angle), std::sin(angle));
}

}  // namespace ateam_geometry

#endif  // ATEAM_GEOMETRY__CREATION_HELPERS_HPP_
