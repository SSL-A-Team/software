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

#ifndef ATEAM_GEOMETRY__UTILITIES_HPP_
#define ATEAM_GEOMETRY__UTILITIES_HPP_

#include <Eigen/Dense>
#include <math.h>

namespace ateam_geometry
{
/* We define the 2D cross product here between two vectors w and v
as w_x * v_y - w_y * w_x
This is used for determining whether two line segments intersect

The implementation used is based off of this StackOverflow post
https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect*/
double cross_product_2d(const Eigen::Vector2d & w, const Eigen::Vector2d & v)
{
  return (w.x() * v.y()) - (w.y() * v.x());
}

/* Sort Eigen::Vector types starting with index 0 up to the max index.
Basically if in a dimension x > y, then return true,
else, check the next dimension (unless it doesn't exist)
*/
bool sort_eigen_2d_high_low(const Eigen::Vector2d &a, const Eigen::Vector2d &b)
{
  bool a_higher = false;
  for (size_t i = 0; i < a.size(); ++i){
    if (a[i] > b[i]) {
      a_higher = true;
      return a_higher;
    } else if (a[i] < b[i]){
      a_higher = false;
      return a_higher;
    }
  }
  return a_higher;
}

}  // namespace ateam_geometry

#endif  // ATEAM_GEOMETRY__UTILITIES_HPP_
