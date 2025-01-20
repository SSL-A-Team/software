// Copyright 2025 A Team
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

#include "ateam_geometry/angles.hpp"

TEST(AnglesTests, IsClockwiseBetween)
{
  EXPECT_TRUE(ateam_geometry::IsClockwiseBetween(2.0, 1.0, 3.0));
  EXPECT_FALSE(ateam_geometry::IsClockwiseBetween(0.5, 1.0, 3.0));
  EXPECT_TRUE(ateam_geometry::IsClockwiseBetween(0.1, 3.0, 1.0));
  EXPECT_FALSE(ateam_geometry::IsClockwiseBetween(1.5, 2.5, 0.5));
}

TEST(AnglesTests, IsCounterclockwiseBetween)
{
  EXPECT_FALSE(ateam_geometry::IsCounterclockwiseBetween(2.0, 1.0, 3.0));
  EXPECT_TRUE(ateam_geometry::IsCounterclockwiseBetween(0.5, 1.0, 3.0));
  EXPECT_FALSE(ateam_geometry::IsCounterclockwiseBetween(0.1, 3.0, 1.0));
  EXPECT_TRUE(ateam_geometry::IsCounterclockwiseBetween(1.5, 2.5, 0.5));
}
