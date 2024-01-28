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

#ifndef ATEAM_GEOMETRY__NORMALIZE_HPP_
#define ATEAM_GEOMETRY__NORMALIZE_HPP_

#include <Eigen/Dense>
#include "ateam_geometry/types.hpp"


namespace ateam_geometry
{

template<typename T>
inline auto normalize(T const & V)
{
  auto const slen = V.squared_length();
  auto const d = CGAL::approximate_sqrt(slen);
  return d > 0 ? (V / d) : V;
}


template<typename T, typename U>
inline double norm(T const & V, U const & C)
{
  return CGAL::approximate_sqrt(CGAL::squared_distance(V, C));
}

template<typename T>
inline double norm(T const & V)
{
  return CGAL::approximate_sqrt(V.squared_length());
}

}  // namespace ateam_geometry


#endif  // ATEAM_GEOMETRY__NORMALIZE_HPP_
