// Copyright 2021 A Team
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

#include "ateam_common/angle.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace geometry = ateam_common::geometry;

TEST(Angle, WrapToNPiPi)
{
  // No wrap
  EXPECT_NEAR(geometry::WrapToNPiPi(-M_PI), -M_PI, 1e-5);
  EXPECT_NEAR(geometry::WrapToNPiPi(-3), -3, 1e-5);
  EXPECT_NEAR(geometry::WrapToNPiPi(-2), -2, 1e-5);
  EXPECT_NEAR(geometry::WrapToNPiPi(-1), -1, 1e-5);
  EXPECT_NEAR(geometry::WrapToNPiPi(0), 0, 1e-5);
  EXPECT_NEAR(geometry::WrapToNPiPi(1), 1, 1e-5);
  EXPECT_NEAR(geometry::WrapToNPiPi(2), 2, 1e-5);
  EXPECT_NEAR(geometry::WrapToNPiPi(3), 3, 1e-5);
  EXPECT_NEAR(geometry::WrapToNPiPi(M_PI), M_PI, 1e-5);

  // Wrap up
  EXPECT_NEAR(geometry::WrapToNPiPi(-6 * M_PI), 0, 1e-5);
  EXPECT_NEAR(geometry::WrapToNPiPi(-5 * M_PI), -M_PI, 1e-5);
  EXPECT_NEAR(geometry::WrapToNPiPi(-4 * M_PI), 0, 1e-5);
  EXPECT_NEAR(geometry::WrapToNPiPi(-3 * M_PI), -M_PI, 1e-5);
  EXPECT_NEAR(geometry::WrapToNPiPi(-4), -4 + 2 * M_PI, 1e-5);

  // Wrap down
  EXPECT_NEAR(geometry::WrapToNPiPi(6 * M_PI), 0, 1e-5);
  EXPECT_NEAR(geometry::WrapToNPiPi(5 * M_PI), M_PI, 1e-5);
  EXPECT_NEAR(geometry::WrapToNPiPi(4 * M_PI), 0, 1e-5);
  EXPECT_NEAR(geometry::WrapToNPiPi(3 * M_PI), M_PI, 1e-5);
  EXPECT_NEAR(geometry::WrapToNPiPi(4), 4 - 2 * M_PI, 1e-5);
}

TEST(Angle, SignedSmallestAngleDifferent)
{
  // Positive difference no wrap
  EXPECT_NEAR(geometry::SignedSmallestAngleDifference(3, 2), 1, 1e-5);
  EXPECT_NEAR(geometry::SignedSmallestAngleDifference(3, 1), 2, 1e-5);
  EXPECT_NEAR(geometry::SignedSmallestAngleDifference(3, 0), 3, 1e-5);
  EXPECT_NEAR(geometry::SignedSmallestAngleDifference(2, 1), 1, 1e-5);
  EXPECT_NEAR(geometry::SignedSmallestAngleDifference(2, 0), 2, 1e-5);
  EXPECT_NEAR(geometry::SignedSmallestAngleDifference(2, -1), 3, 1e-5);
  EXPECT_NEAR(geometry::SignedSmallestAngleDifference(1, 0), 1, 1e-5);
  EXPECT_NEAR(geometry::SignedSmallestAngleDifference(1, -1), 2, 1e-5);
  EXPECT_NEAR(geometry::SignedSmallestAngleDifference(1, -2), 3, 1e-5);
  EXPECT_NEAR(geometry::SignedSmallestAngleDifference(0, -1), 1, 1e-5);
  EXPECT_NEAR(geometry::SignedSmallestAngleDifference(0, -2), 2, 1e-5);
  EXPECT_NEAR(geometry::SignedSmallestAngleDifference(0, -3), 3, 1e-5);


  // Negative difference no wrap
  EXPECT_NEAR(geometry::SignedSmallestAngleDifference(2, 3), -1, 1e-5);
  EXPECT_NEAR(geometry::SignedSmallestAngleDifference(1, 3), -2, 1e-5);
  EXPECT_NEAR(geometry::SignedSmallestAngleDifference(0, 3), -3, 1e-5);
  EXPECT_NEAR(geometry::SignedSmallestAngleDifference(1, 2), -1, 1e-5);
  EXPECT_NEAR(geometry::SignedSmallestAngleDifference(0, 2), -2, 1e-5);
  EXPECT_NEAR(geometry::SignedSmallestAngleDifference(-1, 2), -3, 1e-5);
  EXPECT_NEAR(geometry::SignedSmallestAngleDifference(0, 1), -1, 1e-5);
  EXPECT_NEAR(geometry::SignedSmallestAngleDifference(-1, 1), -2, 1e-5);
  EXPECT_NEAR(geometry::SignedSmallestAngleDifference(-2, 1), -3, 1e-5);
  EXPECT_NEAR(geometry::SignedSmallestAngleDifference(-1, 0), -1, 1e-5);
  EXPECT_NEAR(geometry::SignedSmallestAngleDifference(-2, 0), -2, 1e-5);
  EXPECT_NEAR(geometry::SignedSmallestAngleDifference(-3, 0), -3, 1e-5);

  // Positive difference wrap
  EXPECT_NEAR(geometry::SignedSmallestAngleDifference(-3, 3), 2 * M_PI - 6, 1e-5);

  // Negative difference wrap
  EXPECT_NEAR(geometry::SignedSmallestAngleDifference(3, -3), -(2 * M_PI - 6), 1e-5);
}
