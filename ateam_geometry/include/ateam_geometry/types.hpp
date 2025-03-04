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

#ifndef ATEAM_GEOMETRY__TYPES_HPP_
#define ATEAM_GEOMETRY__TYPES_HPP_

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polygon_2.h>

namespace ateam_geometry
{
using Kernel = CGAL::Simple_cartesian<double>;
using Point = Kernel::Point_2;
using Segment = Kernel::Segment_2;
using Ray = Kernel::Ray_2;
using Rectangle = Kernel::Iso_rectangle_2;
using Circle = Kernel::Circle_2;
using Line = Kernel::Line_2;
using Polygon = CGAL::Polygon_2<Kernel>;
using Vector = Kernel::Vector_2;
using Direction = Kernel::Direction_2;
}  // namespace ateam_geometry

#endif  // ATEAM_GEOMETRY__TYPES_HPP_
