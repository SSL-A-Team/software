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

using ateam_ai::trajectory_generation::RrtPathPlanner;

TEST(RrtPathPlannerTests, basicTest) {
  RrtPathPlanner planner;

  World world;
  Eigen::Vector3d start_pos;
  Eigen::Vector3d start_vel;
  Eigen::Vector3d goal_pos;
  goal_pos.x() = 1.0;
  Eigen::Vector3d goal_vel;
  double time_step_size = 0.01;

  auto trajectory = planner.generatePath(world, start_pos, start_vel, goal_pos, goal_vel, time_step_size);

  for (const auto & sample : trajectory.samples) {
    std::cout << sample.pose.x() << ", " << sample.pose.y() << '\n';
  }

  EXPECT_FALSE(trajectory.samples.empty());

  FAIL();
}
