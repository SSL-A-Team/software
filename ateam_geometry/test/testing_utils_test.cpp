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

#include <gtest/gtest.h>

#include "ateam_geometry/types.hpp"
#include "ateam_geometry_testing/testing_utils.hpp"

using ateam_geometry::Point;

TEST(PointIsNearTests, MatchNearPoints)
{
  EXPECT_THAT(Point(1, 1), PointIsNear(Point(1, 1)));
  EXPECT_THAT(Point(1, 1), PointIsNear(Point(1.005, 1.005)));
}

TEST(PointIsNearTests, DontMatchFarPoints)
{
  EXPECT_THAT(Point(0, 0), testing::Not(PointIsNear(Point(2, 2))));
}

TEST(PointIsNearTests, AllowChangingThreshold)
{
  EXPECT_THAT(Point(0, 0), PointIsNear(Point(2, 2), 3.0));
  EXPECT_THAT(Point(0, 0), testing::Not(PointIsNear(Point(4, 4), 3.0)));
}

TEST(PointsAreNearTests, MatchNearPoints)
{
  std::vector<Point> points = {{0, 0}, {5, 5}};
  EXPECT_THAT(points, testing::Pointwise(PointsAreNear(), points));
}

TEST(PointsAreNearTests, DontMatchFarPoints)
{
  std::vector<Point> p1 = {{1, 2}, {3, 4}};
  std::vector<Point> p2 = {{5, 6}, {7, 8}};
  EXPECT_THAT(p1, testing::Pointwise(testing::Not(PointsAreNear()), p2));
}

TEST(PointsAreNearTests, AllowChangingThreshold)
{
  std::vector<Point> p1 = {{1, 2}, {3, 4}};
  std::vector<Point> p2 = {{5, 6}, {7, 8}};
  EXPECT_THAT(p1, testing::Pointwise(PointsAreNear(10.0), p2));
}
