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
#include <gmock/gmock.h>

#include "ateam_common/km_assignment.hpp"

namespace assignment = ateam_common::assignment;

TEST(Assignment, optimize_assignment) {
  Eigen::Matrix<double, 3, 3> cost3x3{{10, 4, 7}, {5, 8, 3}, {1, 4, 9}};
  auto out3x3 = km_assignment::max_cost_assignment(cost3x3);
  EXPECT_EQ(out3x3.size(), 3);
  EXPECT_EQ(out3x3.at(0), 0);
  EXPECT_EQ(out3x3.at(1), 1);
  EXPECT_EQ(out3x3.at(2), 2);

  // Eigen::Matrix<double, 5, 5> cost5x5{
  //   {10, 5, 13, 15, 16},
  //   {3, 9, 18, 13, 6},
  //   {10, 7, 2, 2, 2},
  //   {7, 11, 9, 7, 12},
  //   {7, 9, 10, 4, 12}
  // };
  // auto out5x5 = assignment::optimize_assignment(cost5x5);
  // EXPECT_EQ(out5x5.size(), 5);
  // EXPECT_EQ(out5x5.at(0), 1);
  // EXPECT_EQ(out5x5.at(1), 0);
  // EXPECT_EQ(out5x5.at(2), 4);
  // EXPECT_EQ(out5x5.at(3), 2);
  // EXPECT_EQ(out5x5.at(4), 3);

  // Eigen::Matrix<double, 3, 2> cost3x2{
  //   {1, 10},
  //   {10, 1},
  //   {5, 5}
  // };
  // auto out3x2 = assignment::optimize_assignment(cost3x2);
  // EXPECT_EQ(out3x2.size(), 2);
  // EXPECT_EQ(out3x2.at(0), 0);
  // EXPECT_EQ(out3x2.at(1), 1);

  // Eigen::Matrix<double, 2, 3> cost2x3{
  //   {1, 10, 5},
  //   {10, 1, 5},
  // };
  // auto out2x3 = assignment::optimize_assignment(cost2x3);
  // EXPECT_EQ(out2x3.size(), 2);
  // EXPECT_EQ(out2x3.at(0), 0);
  // EXPECT_EQ(out2x3.at(1), 1);

  // Eigen::Matrix<double, 8, 1> cost8x1{{3.1}, {3.09}, {3.64}, {3.44}, {3.47}, {3.44}, {3.85},
  //   {3.44}};
  // auto out8x1 = assignment::optimize_assignment(cost8x1.transpose());
  // EXPECT_EQ(out8x1.size(), 1);
  // EXPECT_EQ(out8x1.at(0), 1);
}