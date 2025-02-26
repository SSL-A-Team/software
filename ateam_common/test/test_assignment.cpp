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

#include "ateam_common/assignment/assign_tasks.hpp"

namespace assignment = ateam_common::assignment;

using ::testing::ElementsAre;

TEST(Assignment, empty_costs) {
  Eigen::Matrix<double, 0, 0> costs{};
  const auto out = assignment::AssignTasks(costs);
  EXPECT_THAT(out, ElementsAre());
}

TEST(Assignment, trivial_assignment) {
  Eigen::Matrix<double, 3, 3> cost3x3{
    {10, 4, 7},
    {5, 8, 3},
    {1, 4, 9}
  };
  std::vector<int> out3x3 = assignment::AssignTasks(cost3x3);
  EXPECT_THAT(out3x3, ElementsAre(0, 1, 2));

  Eigen::Matrix<double, 5, 5> cost5x5{
    {20, 5, 13, 15, 16},
    {3, 9, 18, 13, 6},
    {10, 19, 2, 2, 2},
    {7, 11, 9, 17, 12},
    {7, 9, 10, 4, 15}
  };
  std::vector<int> out5x5 = assignment::AssignTasks(cost5x5);
  EXPECT_THAT(out5x5, ElementsAre(0, 2, 1, 3, 4));
}

TEST(Assignment, negative_cost_assignment) {
  Eigen::Matrix<double, 3, 3> cost3x3{
    {-10, -4, 0},
    {1, 0, -5},
    {0, 4, -10}
  };
  std::vector<int> out3x3 = assignment::AssignTasks(cost3x3);
  EXPECT_THAT(out3x3, ElementsAre(2, 0, 1));

  Eigen::Matrix<double, 2, 2> cost2x2{
    {-10.02, -4},
    {1, 0.005}
  };
  std::vector<int> out2x2 = assignment::AssignTasks(cost2x2);
  EXPECT_THAT(out2x2, ElementsAre(1, 0));
}

TEST(Assignment, non_square_assignment) {
  Eigen::Matrix<double, 3, 2> cost3x2{
    {1, 10},
    {10, 1},
    {5, 5}
  };
  auto out3x2 = assignment::AssignTasks(cost3x2);
  EXPECT_THAT(out3x2, ElementsAre(1, 0, -1));

  Eigen::Matrix<double, 2, 3> cost2x3{
    {1, 10, 5},
    {10, 1, 5},
  };
  auto out2x3 = assignment::AssignTasks(cost2x3);
  EXPECT_THAT(out2x3, ElementsAre(1, 0));

  Eigen::Matrix<double, 2, 5> cost2x5{
    {1, 10, 5, 2, 3},
    {10, 1, 5, 2, 3},
  };
  auto out2x5 = assignment::AssignTasks(cost2x3);
  EXPECT_THAT(out2x5, ElementsAre(1, 0));

  Eigen::Matrix<double, 2, 1> cost2x1{
    {2.8},
    {0}
  };
  auto out2x1 = assignment::AssignTasks(cost2x1, assignment::AssignmentType::MinCost);
  EXPECT_THAT(out2x1, ElementsAre(-1, 0));
}

TEST(Assignment, min_cost_assignment) {
  Eigen::Matrix<double, 3, 3> cost3x3{
    {-10, -4, 0},
    {1, 0, -5},
    {0, 4, -10}
  };
  std::vector<int> out3x3 = assignment::AssignTasks(
    cost3x3, assignment::AssignmentType::MinCost);
  EXPECT_THAT(out3x3, ElementsAre(0, 1, 2));

  Eigen::Matrix<double, 2, 2> cost2x2{
    {-10.02, -4},
    {1, 0.005}
  };
  std::vector<int> out2x2 = assignment::AssignTasks(
    cost2x2, assignment::AssignmentType::MinCost);
  EXPECT_THAT(out2x2, ElementsAre(0, 1));
}

TEST(Assignment, handle_nan) {
  Eigen::Matrix<double, 3, 3> cost3x3{
    {-10, -4, 0},
    {1, 0, -5},
    {NAN, 4, -10}
  };
  std::vector<int> out3x3 = assignment::AssignTasks(cost3x3);
  EXPECT_THAT(out3x3, ElementsAre(2, 0, 1));

  Eigen::Matrix<double, 2, 2> cost2x2{
    {NAN, -4},
    {1, 0.005}
  };
  std::vector<int> out2x2 = assignment::AssignTasks(cost2x2);
  EXPECT_THAT(out2x2, ElementsAre(1, 0));
}

TEST(Assignment, basic_forbidden_assignments) {
  Eigen::Matrix<double, 3, 3> cost3x3{
    {13, 12, 7},
    {11, 8, 3},
    {1, 4, 9}
  };
  std::map<int, std::vector<int>> forbidden{{0, std::vector<int>{0}}};
  std::vector<int> out3x3 = assignment::AssignTasks(
    cost3x3,
    assignment::AssignmentType::MaxCost,
    forbidden);
  EXPECT_THAT(out3x3, ElementsAre(1, 0, 2));
}

TEST(Assignment, all_forbidden_assignments) {
  Eigen::Matrix<double, 3, 3> cost3x3{
    {13, 12, 7},
    {11, 8, 3},
    {1, 4, 9}
  };
  std::map<int, std::vector<int>> forbidden_col{{0, std::vector<int>{0}},
    {1, std::vector<int>{0}},
    {2, std::vector<int>{0}}};
  std::vector<int> out_col = assignment::AssignTasks(
    cost3x3,
    assignment::AssignmentType::MaxCost,
    forbidden_col);
  EXPECT_THAT(out_col, ElementsAre(1, -1, 2));

  std::map<int, std::vector<int>> forbidden_all{{0, std::vector<int>{0, 1, 2}},
    {1, std::vector<int>{0, 1, 2}},
    {2, std::vector<int>{0, 1, 2}}};
  std::vector<int> out_all = assignment::AssignTasks(
    cost3x3,
    assignment::AssignmentType::MaxCost,
    forbidden_all);
  EXPECT_THAT(out_all, ElementsAre(-1, -1, -1));

  std::map<int, std::vector<int>> forbidden_row{{0, std::vector<int>{0, 1, 2}}};
  std::vector<int> out_row = assignment::AssignTasks(
    cost3x3,
    assignment::AssignmentType::MaxCost,
    forbidden_row);
  EXPECT_THAT(out_row, ElementsAre(-1, 0, 2));
}

TEST(Assignment, swap_passer_receiver_example) {
  Eigen::Matrix<double, 2, 2> cost{
    {1.75057, 0.138011},
    {0.0, 1.85326}
  };
  std::map<int, std::vector<int>> forbidden{
    {1, std::vector<int>{0}}
  };
  std::vector<int> result = assignment::AssignTasks(
    cost,
    assignment::AssignmentType::MinCost,
    forbidden);
  EXPECT_THAT(result, testing::ElementsAre(0, 1));
}
