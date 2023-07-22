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

#include <angles/angles.h>
#include <Eigen/Dense>
#include <cmath>
#include <ateam_geometry/types.hpp>


namespace ateam_common
{
namespace geometry
{

inline double VectorToAngle(const Eigen::Vector2d & vector)
{
  return atan2(vector.y(), vector.x());
}

inline bool IsVectorAligned(
  const Eigen::Vector2d & a, const Eigen::Vector2d & b,
  double max_angle_diff = 0.1)
{
  double a_angle = VectorToAngle(a);
  double b_angle = VectorToAngle(b);
  return std::abs(angles::shortest_angular_distance(a_angle, b_angle)) < max_angle_diff;
}

// as a concept points and vectors are interchangle for having 2 coeffs so this would work here
template<typename T>
inline double VectorToAngle(const T & vector)
{
  return atan2(vector.y(), vector.x());
}


template<typename T, typename U>
inline bool IsVectorAligned(
  const T & a, const U & b,
  double max_angle_diff = 0.1)
{
  double a_angle = VectorToAngle(a);
  double b_angle = VectorToAngle(b);
  return std::abs(angles::shortest_angular_distance(a_angle, b_angle)) < max_angle_diff;
}

}  // namespace geometry
}  // namespace ateam_common

#endif  // ATEAM_COMMON__ANGLE_HPP_
