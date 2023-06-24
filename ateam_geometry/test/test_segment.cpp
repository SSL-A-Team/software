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

#include "ateam_geometry/segment.hpp"

namespace geometry = ateam_geometry;

TEST(Segment, points_on_segment) {
  geometry::LineSegment test_segment = geometry::LineSegment(
    Eigen::Vector2d(
      0,
      0), Eigen::Vector2d(
      2, 2));

  auto point_on_line = Eigen::Vector2d(1, 1);
  EXPECT_TRUE(geometry::is_point_on_segment(point_on_line, test_segment));

  auto endpoint = Eigen::Vector2d(0, 0);
  EXPECT_TRUE(geometry::is_point_on_segment(endpoint, segment));

  auto too_far = Eigen::Vector2d(3, 3);
  EXPECT_FALSE(geometry::is_point_on_segment(too_far, segment));

  auto also_too_far = Eigen::Vector2d(-1, -1);
  EXPECT_FALSE(geometry::is_point_on_segment(also_too_far, segment));

  auto not_on_line = Eigen::Vector2d(1, 2);
  EXPECT_FALSE(geometry::is_point_on_segment(not_on_line, segment));
}

TEST(Segment, intersection) {
  // Initialize using 2 endpoints
  geometry::LineSegment segment1 =
    geometry::LineSegment(Eigen::Vector2d(-1, -1), Eigen::Vector2d(1, 1));
  geometry::LineSegment segment2 =
    geometry::LineSegment(Eigen::Vector2d(-1, 1), Eigen::Vector2d(1, -1));
  // Initialize with starting point, angle, and length
  geometry::LineSegment segment3 = geometry::LineSegment(
    Eigen::Vector2d(-1, -1), 2 * sqrt(
      2), M_PI / 4);
  geometry::LineSegment segment4 = geometry::LineSegment(
    Eigen::Vector2d(-1, 1), 2 * sqrt(
      2), 7 * M_PI / 4);

  EXPECT_THAT(geometry::get_segment_intersection(segment1, segment2), Eq(Eigen::Vector2d(0, 0)));
  EXPECT_THAT(geometry::get_segment_intersection(segment1, segment4), Eq(Eigen::Vector2d(0, 0)));
  EXPECT_THAT(geometry::get_segment_intersection(segment3, segment4), Eq(Eigen::Vector2d(0, 0)));
  EXPECT_THAT(geometry::get_segment_intersection(segment3, segment2), Eq(Eigen::Vector2d(0, 0)));

  EXPECT_THAT(geometry::get_segment_intersection(segment2, segment4), Eq(Eigen::Vector2d(-1, 1)));
  EXPECT_THAT(geometry::get_segment_intersection(segment1, segment3), Eq(Eigen::Vector2d(-1, -1)));

  geometry::LineSegment doesnt_intersect = geometry::LineSegment(
    Eigen::Vector2d(
      3,
      3), Eigen::Vector2d(
      5, 5));
  EXPECT_THAT(geometry::get_segment_intersection(segment1, doesnt_intersect), Eq(std::nullopt));
  EXPECT_THAT(geometry::get_segment_intersection(segment2, doesnt_intersect), Eq(std::nullopt));
}
