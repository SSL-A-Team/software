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

#include <gtest/gtest.h>

#include <cmath>

#include "ateam_common/angle.hpp"

namespace geometry = ateam_common::geometry;

TEST(Angle, IsVectorAligned)
{
  EXPECT_TRUE(geometry::IsVectorAligned(Eigen::Vector2d{1, 0}, Eigen::Vector2d{1, 0}, 0.1));
  EXPECT_TRUE(geometry::IsVectorAligned(Eigen::Vector2d{0, 1}, Eigen::Vector2d{1, 0}, 0.75 * M_PI));
  EXPECT_TRUE(geometry::IsVectorAligned(Eigen::Vector2d{1, 0}, Eigen::Vector2d{0, 1}, 0.75 * M_PI));

  EXPECT_FALSE(geometry::IsVectorAligned(Eigen::Vector2d{0, 1}, Eigen::Vector2d{1, 0}, 0.1));
  EXPECT_FALSE(geometry::IsVectorAligned(Eigen::Vector2d{1, 0}, Eigen::Vector2d{0, 1}, 0.1));
}
