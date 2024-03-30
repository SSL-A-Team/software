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

#include "ateam_common/km_assignment.hpp"

namespace km_assignment = ateam_common::km_assignment;

TEST(KmAssignment, trivial_assignment) {
  Eigen::Matrix<double, 3, 3> cost3x3{
    {10, 4, 7},
    {5, 8, 3},
    {1, 4, 9}
  };
  std::vector<int> out3x3 = km_assignment::max_cost_assignment(cost3x3);
  EXPECT_EQ(out3x3.size(), 3);
  EXPECT_EQ(out3x3.at(0), 0);
  EXPECT_EQ(out3x3.at(1), 1);
  EXPECT_EQ(out3x3.at(2), 2);

  Eigen::Matrix<double, 5, 5> cost5x5{
    {20, 5, 13, 15, 16},
    {3, 9, 18, 13, 6},
    {10, 19, 2, 2, 2},
    {7, 11, 9, 17, 12},
    {7, 9, 10, 4, 15}
  };
  std::vector<int> out5x5 = km_assignment::max_cost_assignment(cost5x5);
  EXPECT_EQ(out5x5.size(), 5);
  EXPECT_EQ(out5x5.at(0), 0);
  EXPECT_EQ(out5x5.at(1), 2);
  EXPECT_EQ(out5x5.at(2), 1);
  EXPECT_EQ(out5x5.at(3), 3);
  EXPECT_EQ(out5x5.at(4), 4);
}

TEST(KmAssignment, negative_cost_assignment) {
  Eigen::Matrix<double, 3, 3> cost3x3{
    {-10, -4, 0},
    {1, 0, -5},
    {0, 4, -10}
  };
  std::vector<int> out3x3 = km_assignment::max_cost_assignment(cost3x3);
  EXPECT_EQ(out3x3.size(), 3);
  EXPECT_EQ(out3x3.at(0), 2);
  EXPECT_EQ(out3x3.at(1), 0);
  EXPECT_EQ(out3x3.at(2), 1);

  Eigen::Matrix<double, 2, 2> cost2x2{
    {-10.02, -4},
    {1, 0.005}
  };
  std::vector<int> out2x2 = km_assignment::max_cost_assignment(cost2x2);
  EXPECT_EQ(out2x2.size(), 2);
  EXPECT_EQ(out2x2.at(0), 1);
  EXPECT_EQ(out2x2.at(1), 0);
}

TEST(KmAssignment, non_square_assignment) {
  Eigen::Matrix<double, 3, 2> cost3x2{
    {1, 10},
    {10, 1},
    {5, 5}
  };
  auto out3x2 = km_assignment::max_cost_assignment(cost3x2);
  EXPECT_EQ(out3x2.size(), 3);
  EXPECT_EQ(out3x2.at(0), 1);
  EXPECT_EQ(out3x2.at(1), 0);
  EXPECT_EQ(out3x2.at(2), -1);

  Eigen::Matrix<double, 2, 3> cost2x3{
    {1, 10, 5},
    {10, 1, 5},
  };
  auto out2x3 = km_assignment::max_cost_assignment(cost2x3);
  EXPECT_EQ(out2x3.size(), 3);
  EXPECT_EQ(out2x3.at(0), 1);
  EXPECT_EQ(out2x3.at(1), 0);
}

TEST(KmAssignment, min_cost_assignment) {
  Eigen::Matrix<double, 3, 3> cost3x3{
    {-10, -4, 0},
    {1, 0, -5},
    {0, 4, -10}
  };
  std::vector<int> out3x3 = km_assignment::max_cost_assignment(
    cost3x3, false);
  EXPECT_EQ(out3x3.size(), 3);
  EXPECT_EQ(out3x3.at(0), 0);
  EXPECT_EQ(out3x3.at(1), 1);
  EXPECT_EQ(out3x3.at(2), 2);

  Eigen::Matrix<double, 2, 2> cost2x2{
    {-10.02, -4},
    {1, 0.005}
  };
  std::vector<int> out2x2 = km_assignment::max_cost_assignment(
    cost2x2, false);
  EXPECT_EQ(out2x2.size(), 2);
  EXPECT_EQ(out2x2.at(0), 0);
  EXPECT_EQ(out2x2.at(1), 1);
}

TEST(KmAssignment, handle_nan) {
  Eigen::Matrix<double, 3, 3> cost3x3{
    {-10, -4, 0},
    {1, 0, -5},
    {NAN, 4, -10}
  };
  std::vector<int> out3x3 = km_assignment::max_cost_assignment(cost3x3);
  EXPECT_EQ(out3x3.size(), 3);
  EXPECT_EQ(out3x3.at(0), 2);
  EXPECT_EQ(out3x3.at(1), 0);
  EXPECT_EQ(out3x3.at(2), 1);

  Eigen::Matrix<double, 2, 2> cost2x2{
    {NAN, -4},
    {1, 0.005}
  };
  std::vector<int> out2x2 = km_assignment::max_cost_assignment(cost2x2);
  EXPECT_EQ(out2x2.size(), 2);
  EXPECT_EQ(out2x2.at(0), 1);
  EXPECT_EQ(out2x2.at(1), 0);
}
