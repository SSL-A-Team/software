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

#include "ateam_common/robot_assignment.hpp"

using assign = ateam_common::robot_assignment::assign;

TEST(RobotAssignment, assign_simple) {
  // goals should be assigned in robot order as the robots are on top of each goal
  std::vector<Robot> robots1 {{
    {{1, 0, 0}, {}, {}, 0},
    {{2, 0, 0}, {}, {}, 1},
    {{3, 0, 0}, {}, {}, 2},
    {{4, 0, 0}, {}, {}, 3}
  }};

  std::vector<Eigen::Vector2d> goals1 {{
    {1, 0},
    {2, 0},
    {3, 0},
    {4, 0}
  }};

  std::map<size_t, size_t> exp_result1 {{
    {0, 0},
    {1, 1},
    {2, 2},
    {3, 3},
  }};
  auto result1 = assign(robots1, goals1);
  EXPECT_EQ(result1, exp_result1);
}

TEST(RobotAssignment, less_goals_than_robots) {
  // less goals than robots
  std::vector<Robot> robots2 {{
    {{1, 0, 0}, {}, {}, 0},
    {{2, 0, 0}, {}, {}, 1},
    {{3, 0, 0}, {}, {}, 2},
    {{4, 0, 0}, {}, {}, 3}
  }};

  std::vector<Eigen::Vector2d> goals2 {{
    {1, 0},
    {2, 0},
  }};

  std::map<size_t, size_t> exp_result2 {{
    {0, 0},
    {1, 1},
  }};

  auto result2 = assign(robots2, goals2);
  EXPECT_EQ(result2, exp_result2);
}

TEST(RobotAssignment, more_goals_than_robots) {
  // More goals than robots
  std::vector<Robot> robots3 {{
    {{3, 0, 0}, {}, {}, 0},
    {{4, 0, 0}, {}, {}, 1},
  }};

  std::vector<Eigen::Vector2d> goals3 {{
    {1, 0},
    {2, 0},
    {3, 0},
    {4, 0}
  }};

  std::map<size_t, size_t> exp_result3 {{
    {0, 2},
    {1, 3},
  }};

  auto result3 = assign(robots3, goals3);
  EXPECT_EQ(result3, exp_result3);
}
