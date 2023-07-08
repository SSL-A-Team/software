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

#ifndef ATEAM_GEOMETRY__NEAREST_POINTS_HPP_
#define ATEAM_GEOMETRY__NEAREST_POINTS_HPP_

#include "ateam_geometry/types.hpp"

namespace ateam_geometry
{

inline ateam_geometry::Point NearestPointOnSegment(
  const ateam_geometry::Segment & s, const ateam_geometry::Point & p)
{
  ateam_geometry::Point orthogonal_projection = s.supporting_line().projection(p);
  if (s.has_on(orthogonal_projection)) {
    return orthogonal_projection;
  }
  return CGAL::squared_distance(orthogonal_projection, s.source()) <
         CGAL::squared_distance(orthogonal_projection, s.target()) ?
         s.source() : s.target();
}


// Thinking on it basically what I am about to do is GJK. Either the shapes intersect
/*ateam_geometry::Point NearestPoints(AnyShape shape1 , AnyShape shape2)
{

}*/


}  // namespace ateam_geometry

#endif  // ATEAM_GEOMETRY__NEAREST_POINTS_HPP_
