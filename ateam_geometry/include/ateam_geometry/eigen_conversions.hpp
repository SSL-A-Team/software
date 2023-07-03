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

#ifndef ATEAM_GEOMETRY__EIGEN_CONVERSIONS_HPP_
#define ATEAM_GEOMETRY__EIGEN_CONVERSIONS_HPP_

#include <Eigen/Dense>
#include "types.hpp"

namespace ateam_geometry
{

/**
 * @brief Convert geometry point to Eigen Vector2d
 *
 * @param p geometry point object
 * @return Eigen::Vector2d
 */
inline Eigen::Vector2d PointToEigen(const Point & p)
{
  return {p.x(), p.y()};
}

/**
 * @brief Convert Eigen Vector2d to geometry point
 *
 * @param p Eigen Vector2d object
 * @return Point
 */
inline Point EigenToPoint(const Eigen::Vector2d & p)
{
  return Point(p.x(), p.y());
}

}  // namespace ateam_geometry

#endif  // ATEAM_GEOMETRY__EIGEN_CONVERSIONS_HPP_
