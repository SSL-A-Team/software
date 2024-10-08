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
#include <gmock/gmock.h>

#include "ateam_geometry/types.hpp"
#include "ateam_geometry/arc.hpp"
#include "ateam_geometry/intersection.hpp"
#include "ateam_geometry/creation_helpers.hpp"
#include "ateam_geometry_testing/testing_utils.hpp"

namespace ag = ateam_geometry;

using ::testing::Eq;
using ::testing::Optional;
using ::testing::VariantWith;
using ::testing::Pointwise;
using ::testing::Pair;

using PointPair = std::pair<ag::Point, ag::Point>;

std::ostream & operator<<(
  std::ostream & stream, const std::optional<std::variant<ag::Point,
  PointPair>> & val)
{
  if (!val) {
    stream << "[empty optional]";
  } else if (std::holds_alternative<ag::Point>(*val)) {
    stream << std::get<ag::Point>(*val);
  } else if (std::holds_alternative<PointPair>(*val)) {
    const auto & pair = std::get<PointPair>(*val);
    stream << "(" << pair.first << ", " << pair.second << ")";
  } else {
    stream << "[bad variant]";
  }
  return stream;
}

TEST(IntersectionTests, CircleToLine)
{
  const auto circle = ag::makeCircle(ag::Point(1, 2), 2);

  const auto line_no_intersect = ag::Line(ag::Point(5, 0), ag::Point(5, 1));
  EXPECT_THAT(ag::intersection(circle, line_no_intersect), Eq(std::nullopt));

  const auto line_tangent = ag::Line(ag::Point(3, 4), ag::Point(4, 4));
  EXPECT_THAT(
    ag::intersection(circle, line_tangent),
    Optional(VariantWith<ag::Point>(PointIsNear(ag::Point(1, 4)))));

  const auto line_secant = ag::Line(ag::Point(0, 1.5), ag::Point(1, 2.5));
  EXPECT_THAT(
    ag::intersection(circle, line_secant),
    Optional(
      VariantWith<PointPair>(
        Pair(
          PointIsNear(ag::Point(-0.6419409, 0.8580589)),
          PointIsNear(ag::Point(2.1419411, 3.6419411))))));

  const auto line_secant_2 = ag::Line(ag::Point(0, 2), ag::Point(1, 2));
  EXPECT_THAT(
    ag::intersection(circle, line_secant_2),
    Optional(
      VariantWith<PointPair>(
        Pair(
          PointIsNear(ag::Point(-1, 2)),
          PointIsNear(ag::Point(3, 2))))));
}

TEST(IntersectionTests, ArcToLine)
{
  const auto arc = ag::Arc(
    ag::Point(10, 10),
    5,
    ag::directionFromAngle(M_PI_2),
    ag::directionFromAngle(-M_PI_2));

  const auto line_no_intersect = ag::Line(ag::Point(10, 0), ag::Point(11, 1));
  EXPECT_THAT(ag::intersection(arc, line_no_intersect), Eq(std::nullopt));

  const auto line_tangent = ag::Line(ag::Point(5, 10), ag::Point(5, 11));
  EXPECT_THAT(
    ag::intersection(arc, line_tangent),
    Optional(VariantWith<ag::Point>(PointIsNear(ag::Point(5, 10)))));

  const auto line_secant = ag::Line(ag::Point(0, 1), ag::Point(2, 4));
  EXPECT_THAT(
    ag::intersection(arc, line_secant),
    Optional(
      VariantWith<PointPair>(
        Pair(
          PointIsNear(ag::Point(5.1609809, 8.7414714)),
          PointIsNear(ag::Point(9.3005575, 14.9508363))))));
}

TEST(IntersectionTests, ArcToRay)
{
  const auto arc =
    ag::Arc(ag::Point(-2, 2), 4, ag::directionFromAngle(0), ag::directionFromAngle(M_PI));

  const auto ray_no_intersect = ag::Ray(ag::Point(0, 0), ag::Point(1, 0));
  EXPECT_THAT(ag::intersection(arc, ray_no_intersect), Eq(std::nullopt));

  const auto ray_tangent = ag::Ray(ag::Point(-2, 6), ag::Point(0, 6));
  EXPECT_THAT(
    ag::intersection(arc, ray_tangent),
    Optional(VariantWith<ag::Point>(PointIsNear(ag::Point(-2, 6)))));

  const auto ray_secant_1_intersect = ag::Ray(ag::Point(0, 1), ag::Point(-2, 4));
  EXPECT_THAT(
    ag::intersection(arc, ray_secant_1_intersect),
    Optional(VariantWith<ag::Point>(PointIsNear(ag::Point(-3.208678, 5.813017)))));

  const auto ray_secant_2_intersects = ag::Ray(ag::Point(-10, 5), ag::Point(0, 5));
  EXPECT_THAT(
    ag::intersection(arc, ray_secant_2_intersects),
    Optional(
      VariantWith<PointPair>(
        Pair(
          PointIsNear(ag::Point(-4.645751, 5)),
          PointIsNear(ag::Point(0.6457513, 5))))));
}

TEST(IntersectionTests, ArcToSegment)
{
  const auto arc =
    ag::Arc(ag::Point(0, 0), 1, ag::directionFromAngle(M_PI), ag::directionFromAngle(M_PI_2));

  const auto segment_no_intersect = ag::Segment(ag::Point(0, 0), ag::Point(-1, 1));
  EXPECT_THAT(ag::intersection(arc, segment_no_intersect), Eq(std::nullopt));

  const auto segment_tangent = ag::Segment(ag::Point(0, -std::sqrt(2)), ag::Point(std::sqrt(2), 0));
  EXPECT_THAT(
    ag::intersection(arc, segment_tangent),
    Optional(VariantWith<ag::Point>(PointIsNear(ag::Point(0.707107, -0.707107)))));

  const auto segment_secant_1_intersect = ag::Segment(ag::Point(0, 0), ag::Point(0, -1));
  EXPECT_THAT(
    ag::intersection(arc, segment_secant_1_intersect),
    Optional(VariantWith<ag::Point>(PointIsNear(ag::Point(0, -1)))));

  const auto segment_secant_2_intersects = ag::Segment(ag::Point(-1, -1), ag::Point(1, 1));
  EXPECT_THAT(
    ag::intersection(arc, segment_secant_2_intersects),
    Optional(
      VariantWith<PointPair>(
        Pair(
          PointIsNear(ag::Point(-0.707107, -0.707107)),
          PointIsNear(ag::Point(0.707107, 0.707107))))));
}

TEST(IntersectionTests, SegmentToSegmentNoIntersection)
{
  ag::Segment a{
    ag::Point{0, 0},
    ag::Point{1, 1}
  };

  ag::Segment b{
    ag::Point{0, 1},
    ag::Point{1, 2}
  };

  EXPECT_THAT(ag::intersection(a, b), Eq(std::nullopt));
}

TEST(IntersectionTests, SegmentToSegmentPointIntersection)
{
  ag::Segment a{
    ag::Point{0, 0},
    ag::Point{1, 1}
  };

  ag::Segment b{
    ag::Point{0, 1},
    ag::Point{1, 0}
  };

  EXPECT_THAT(
    ag::intersection(a, b),
    Optional(VariantWith<ag::Point>(PointIsNear(ag::Point{0.5, 0.5}))));
}

TEST(IntersectionTests, SegmentToSegmentColinearOverlap)
{
  ag::Segment a{
    ag::Point{0, 0},
    ag::Point{1, 1}
  };

  ag::Segment b{
    ag::Point{0.5, 0.5},
    ag::Point{1.5, 1.5}
  };

  EXPECT_THAT(
    ag::intersection(a, b),
    Optional(
      VariantWith<ag::Segment>(
        SegmentIsNear(
          ag::Segment{ag::Point{0.5, 0.5},
            ag::Point{1.0, 1.0}}))));
}

TEST(IntersectionTests, SegmentToSegmentColinearOnePoint)
{
  ag::Segment a{
    ag::Point{0, 0},
    ag::Point{1, 1}
  };

  ag::Segment b{
    ag::Point{1, 1},
    ag::Point{2, 2}
  };

  EXPECT_THAT(
    ag::intersection(a, b),
    Optional(VariantWith<ag::Point>(PointIsNear(ag::Point{1, 1}))));
}

TEST(IntersectionTests, SegmentToSegmentColinearNoIntersection)
{
  ag::Segment a{
    ag::Point{0, 0},
    ag::Point{1, 1}
  };

  ag::Segment b{
    ag::Point{2, 2},
    ag::Point{3, 3}
  };

  EXPECT_THAT(ag::intersection(a, b), Eq(std::nullopt));
}
