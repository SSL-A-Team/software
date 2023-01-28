// Copyright 2021 A Team
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

#ifndef ATEAM_COMMON__ANGLE_HPP_
#define ATEAM_COMMON__ANGLE_HPP_

#include <Eigen/Dense>

#include <cmath>

namespace ateam_common
{
namespace geometry
{

/**
 * @brief Wraps to -PI through PI inclusive
 */
inline double WrapToNPiPi(double a)
{
  return atan2(sin(a), cos(a));
}

/**
 * @brief Returns the smallest angle (a - b) in signed format
 */
inline double SignedSmallestAngleDifference(double a, double b)
{
  double unbound_angle_diff = a - b;
  return atan2(sin(unbound_angle_diff), cos(unbound_angle_diff));
}

inline double VectorToAngle(const Eigen::Vector2d & vector)
{
  return atan2(vector.y(), vector.x());
}

inline double SmallestAngleBetween(const Eigen::Vector2d & a, const Eigen::Vector2d & b)
{
  return acos(a.normalized().dot(b.normalized()));
}

inline bool IsVectorAligned(
  const Eigen::Vector2d & a, const Eigen::Vector2d & b,
  double max_angle_diff = 0.1)
{
  double a_angle = VectorToAngle(a);
  double b_angle = VectorToAngle(b);
  return SignedSmallestAngleDifference(a_angle, b_angle) < max_angle_diff;
}

}  // namespace geometry
}  // namespace ateam_common

#endif  // ATEAM_COMMON__ANGLE_HPP_
