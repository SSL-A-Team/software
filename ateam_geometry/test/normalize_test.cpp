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
#include "ateam_geometry/normalize.hpp"

TEST(Normalize, NormalizeNonZeroVector)
{
  ateam_geometry::Vector v(20, 10);
  auto v_normalized = ateam_geometry::normalize(v);
  EXPECT_FLOAT_EQ(v_normalized.x(), 0.89442718);
  EXPECT_FLOAT_EQ(v_normalized.y(), 0.44721359);
}


TEST(Normalize, NormalizeZeroVector)
{
  ateam_geometry::Vector v(0, 0);
  auto v_normalized = ateam_geometry::normalize(v);
  EXPECT_FLOAT_EQ(v_normalized.x(), 0);
  EXPECT_FLOAT_EQ(v_normalized.y(), 0);
}

TEST(Norm, NormalizedDistanceBetweenPoints)
{
  ateam_geometry::Point p1(10, 0);
  ateam_geometry::Point p2(100, 5);
  EXPECT_FLOAT_EQ(ateam_geometry::norm(p1, p2), 90.138779);
}

TEST(Norm, NormOfVector)
{
  ateam_geometry::Vector v(3, 4);
  EXPECT_FLOAT_EQ(ateam_geometry::norm(v), 5);
}
