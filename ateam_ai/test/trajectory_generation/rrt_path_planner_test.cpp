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

#include "trajectory_generation/rrt_path_planner.hpp"

#include <gtest/gtest.h>
#include <ateam_common/equality_utilities.hpp>
#include <ateam_geometry/ateam_geometry.hpp>

using ateam_ai::trajectory_generation::RrtPathPlanner;

bool vectorsAreNear(const Eigen::Vector2d & a, const Eigen::Vector2d & b)
{
  return ateam_common::allCloseDense(a, b);
}

TEST(RrtPathPlannerTests, basicTest) {
  RrtPathPlanner planner;

  Eigen::Vector2d start_pos(0, 0);
  Eigen::Vector2d goal_pos(1, 0);

  auto path = planner.generatePath({}, {}, start_pos, goal_pos);

  ASSERT_EQ(path.size(), 2u);
  EXPECT_PRED2(vectorsAreNear, path.front(), start_pos);
  EXPECT_PRED2(vectorsAreNear, path.back(), goal_pos);
}

TEST(RrtPathPlannerTests, avoidOneObstacle) {
  RrtPathPlanner planner;

  Eigen::Vector2d start_pos(0, 0);
  Eigen::Vector2d goal_pos(2, 0);

  std::vector<ateam_geometry::AnyShape> obstacles = {
    ateam_geometry::makeCircle(ateam_geometry::Point(1, 0), 0.09)
  };

  auto path = planner.generatePath({}, obstacles, start_pos, goal_pos);

  ASSERT_FALSE(path.empty());
  EXPECT_PRED2(vectorsAreNear, path.front(), start_pos);
  EXPECT_PRED2(vectorsAreNear, path.back(), goal_pos);

  // Expect that no trajectory sample is colliding with the obstacle
  // Using a simple distance check since we know the robot and obstacle are circles
  EXPECT_TRUE(
    std::ranges::none_of(
      path, [](const auto & pos) {
        return (pos - Eigen::Vector2d(1, 0)).norm() < 0.18;
      }));
}
