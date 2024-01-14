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
#include "ateam_geometry/variant_do_intersect.hpp"

TEST(VariantDoIntersect, CircleToPoint)
{
  ateam_geometry::Circle a(ateam_geometry::Point(0, 0), 1);
  ateam_geometry::AnyShape b = ateam_geometry::Point(0.5, 0.5);
  EXPECT_TRUE(ateam_geometry::variantDoIntersect(a, b));

  b = ateam_geometry::Point(1.5, 1.5);
  EXPECT_FALSE(ateam_geometry::variantDoIntersect(a, b));
}

TEST(VariantDoIntersect, CircleToCircle)
{
  ateam_geometry::Circle a(ateam_geometry::Point(0, 0), 1);
  ateam_geometry::AnyShape b = ateam_geometry::Circle(ateam_geometry::Point(0, 0), 0.5);
  EXPECT_TRUE(ateam_geometry::variantDoIntersect(a, b));

  a = ateam_geometry::Circle(ateam_geometry::Point(0, 0), 1);
  b = ateam_geometry::Circle(ateam_geometry::Point(0, 0), 1.5);
  EXPECT_TRUE(ateam_geometry::variantDoIntersect(a, b));

  a = ateam_geometry::Circle(ateam_geometry::Point(0, 0), 1);
  b = ateam_geometry::Circle(ateam_geometry::Point(0, 0), 1);
  EXPECT_TRUE(ateam_geometry::variantDoIntersect(a, b));

  a = ateam_geometry::Circle(ateam_geometry::Point(0, 0), 1);
  b = ateam_geometry::Circle(ateam_geometry::Point(3, 3), 1);
  EXPECT_FALSE(ateam_geometry::variantDoIntersect(a, b));
}

TEST(VariantDoIntersect, CircleToSegment)
{
  ateam_geometry::Circle a(ateam_geometry::Point(0, 0), 1);
  ateam_geometry::AnyShape b = ateam_geometry::Segment(
    ateam_geometry::Point(0, 0),
    ateam_geometry::Point(0.5, 0)
  );
  EXPECT_TRUE(ateam_geometry::variantDoIntersect(a, b));

  b = ateam_geometry::Segment(
    ateam_geometry::Point(2, 0),
    ateam_geometry::Point(0, 0)
  );
  EXPECT_TRUE(ateam_geometry::variantDoIntersect(a, b));

  b = ateam_geometry::Segment(
    ateam_geometry::Point(2, 0),
    ateam_geometry::Point(2, 2.5)
  );
  EXPECT_FALSE(ateam_geometry::variantDoIntersect(a, b));
}

TEST(VariantDoIntersect, CircleToRectangle)
{
  ateam_geometry::Circle a(ateam_geometry::Point(0, 0), 1);
  ateam_geometry::AnyShape b = ateam_geometry::Rectangle(
    ateam_geometry::Point(0, 0),
    ateam_geometry::Point(1, 2)
  );
  EXPECT_TRUE(ateam_geometry::variantDoIntersect(a, b));

  b = ateam_geometry::Rectangle(
    ateam_geometry::Point(-1, -1),
    ateam_geometry::Point(1, 1)
  );
  EXPECT_TRUE(ateam_geometry::variantDoIntersect(a, b));

  b = ateam_geometry::Rectangle(
    ateam_geometry::Point(-2, -2),
    ateam_geometry::Point(-1.5, -1.5)
  );
  EXPECT_FALSE(ateam_geometry::variantDoIntersect(a, b));
}

TEST(VariantDoIntersect, CircleToRay)
{
  ateam_geometry::Circle a(ateam_geometry::Point(0, 0), 1);
  ateam_geometry::AnyShape b = ateam_geometry::Ray(
    ateam_geometry::Point(0, 0),
    ateam_geometry::Point(1, 2)
  );
  EXPECT_TRUE(ateam_geometry::variantDoIntersect(a, b));
  b = ateam_geometry::Ray(
    ateam_geometry::Point(-2, -2),
    ateam_geometry::Point(1, 1)
  );
  EXPECT_TRUE(ateam_geometry::variantDoIntersect(a, b));

  b = ateam_geometry::Ray(
    ateam_geometry::Point(-2, -2),
    ateam_geometry::Point(-3, -3)
  );
  EXPECT_FALSE(ateam_geometry::variantDoIntersect(a, b));
}

TEST(VariantDoIntersect, CircleToCircle)
{
  ateam_geometry::Circle a(ateam_geometry::Point(0, 0), 1);
  ateam_geometry::AnyShape b = ateam_geometry::Circle(ateam_geometry::Point(0, 0), 0.5);
  EXPECT_TRUE(ateam_geometry::variantDoIntersect(a, b));

  a = ateam_geometry::Circle(ateam_geometry::Point(0, 0), 1);
  b = ateam_geometry::Circle(ateam_geometry::Point(0, 0), 1.5);
  EXPECT_TRUE(ateam_geometry::variantDoIntersect(a, b));

  a = ateam_geometry::Circle(ateam_geometry::Point(0, 0), 1);
  b = ateam_geometry::Circle(ateam_geometry::Point(0, 0), 1);
  EXPECT_TRUE(ateam_geometry::variantDoIntersect(a, b));
}

TEST(VariantDoIntersect, CircleToSegment)
{
  ateam_geometry::Circle a(ateam_geometry::Point(0, 0), 1);
  ateam_geometry::AnyShape b = ateam_geometry::Segment(
    ateam_geometry::Point(0, 0),
    ateam_geometry::Point(1, 0)
  );
  EXPECT_TRUE(ateam_geometry::variantDoIntersect(a, b));
}
