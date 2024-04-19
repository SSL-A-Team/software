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

#include <gtest/gtest.h>

#include "ateam_geometry/types.hpp"
#include "ateam_geometry/nearest_point.hpp"
#include "ateam_geometry_testing/testing_utils.hpp"

TEST(NearestPointOnSegment, PointOffSegment)
{
  ateam_geometry::Segment s(ateam_geometry::Point(0, 0), ateam_geometry::Point(10, 10));
  ateam_geometry::Point p(10, 0);
  auto nearest_point = ateam_geometry::nearestPointOnSegment(s, p);
  EXPECT_THAT(nearest_point, PointIsNear(ateam_geometry::Point(5, 5)));
}

TEST(NearestPointOnSegment, PointOnSegment)
{
  ateam_geometry::Segment s(ateam_geometry::Point(0, 0), ateam_geometry::Point(10, 10));
  ateam_geometry::Point p(1, 1);
  auto nearest_point = ateam_geometry::nearestPointOnSegment(s, p);
  EXPECT_THAT(nearest_point, PointIsNear(ateam_geometry::Point(1, 1)));
}
