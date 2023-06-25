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

#include <Eigen/Dense>

#include <gtest/gtest.h>
#include "gmock/gmock.h"

#include "ateam_geometry/utilities.hpp"

namespace geometry = ateam_geometry;

TEST(CrossProduct, cross_product_2d) {
  auto v1 = Eigen::Vector2d(0, 0);
  auto v2 = Eigen::Vector2d(1, 1);
  auto v3 = Eigen::Vector2d(-2, 2);
  auto v4 = Eigen::Vector2d(-1, -1);

  EXPECT_THAT(geometry::cross_product_2d(v1, v2), testing::DoubleEq(0));
  EXPECT_THAT(geometry::cross_product_2d(v2, v3), testing::DoubleEq(4));
  EXPECT_THAT(geometry::cross_product_2d(v3, v2), testing::DoubleEq(-4));
  EXPECT_THAT(geometry::cross_product_2d(v2, v2), testing::DoubleEq(0));
  EXPECT_THAT(geometry::cross_product_2d(v2, v4), testing::DoubleEq(0));
}
