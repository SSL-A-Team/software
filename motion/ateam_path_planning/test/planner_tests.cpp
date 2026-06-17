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
#include <gmock/gmock.h>
#include <ateam_geometry_testing/testing_utils.hpp>
#include "ateam_path_planning/planner.hpp"

using ateam_path_planning::Planner;

TEST(Planner, AllBotsNoTargets) {
  Planner planner;

  const auto paths = planner.PlanPathsForAllBots({}, {}, {}, {}, {});

  EXPECT_THAT(paths, ::testing::Each(::testing::Eq(std::nullopt)));
}

TEST(Planner, OneBotNoObstacles) {
  Planner planner;

  std::array<std::optional<ateam_path_planning::Pose>, 16> targets;
  targets.fill(std::nullopt);
  targets[0] = ateam_path_planning::Pose{ateam_geometry::Point(1.0, 1.0), 0.0};

  std::array<uint8_t, 16> priorities;
  priorities.fill(0);

  ateam_game_state::World world;
  world.our_robots[0].pos = ateam_geometry::Point(0.0, 0.0);
  world.our_robots[0].theta = 0.0;
  world.our_robots[0].vel = ateam_geometry::Vector(0.0, 0.0);

  const auto paths = planner.PlanPathsForAllBots(targets, priorities, world, {}, {});

  ASSERT_TRUE(paths[0].has_value());
  const auto & trajectory = paths[0].value();
  EXPECT_THAT(trajectory.start_pose.position, PointIsNear(ateam_geometry::Point(0.0, 0.0)));
  EXPECT_FLOAT_EQ(trajectory.start_pose.heading, 0.0);
  EXPECT_THAT(trajectory.segments.back().target.position,
    PointIsNear(ateam_geometry::Point(1.0, 1.0)));
  EXPECT_FLOAT_EQ(trajectory.segments.back().target.heading, 0.0);

  for (size_t i = 1; i < paths.size(); ++i) {
    EXPECT_FALSE(paths[i].has_value());
  }
}

TEST(Planner, OneBotOneObstacle) {
  Planner planner;

  std::array<std::optional<ateam_path_planning::Pose>, 16> targets;
  targets.fill(std::nullopt);
  targets[0] = ateam_path_planning::Pose{ateam_geometry::Point(1.0, 1.0), 0.0};

  std::array<uint8_t, 16> priorities;
  priorities.fill(0);

  ateam_game_state::World world;
  world.our_robots[0].pos = ateam_geometry::Point(0.0, 0.0);
  world.our_robots[0].theta = 0.0;
  world.our_robots[0].vel = ateam_geometry::Vector(0.0, 0.0);

  const auto obstacle_shape = ateam_geometry::makeDisk(ateam_geometry::Point(0.5, 0.5), 0.1);

  std::vector<ateam_path_planning::Obstacle> global_obstacles = {
    ateam_path_planning::Obstacle{obstacle_shape, ateam_geometry::Vector(0.0, 0.0)}
  };

  const auto paths = planner.PlanPathsForAllBots(targets, priorities, world, global_obstacles, {});

  ASSERT_TRUE(paths[0].has_value());
  const auto & trajectory = paths[0].value();
  EXPECT_THAT(trajectory.start_pose.position, PointIsNear(ateam_geometry::Point(0.0, 0.0)));
  EXPECT_FLOAT_EQ(trajectory.start_pose.heading, 0.0);
  EXPECT_EQ(trajectory.segments.size(), 2);
  EXPECT_THAT(trajectory.segments.back().target.position,
    PointIsNear(ateam_geometry::Point(1.0, 1.0)));
  EXPECT_FLOAT_EQ(trajectory.segments.back().target.heading, 0.0);

  const auto points = trajectory.ToPoints(0.01);
  for (const auto & point : points) {
    EXPECT_FALSE(ateam_geometry::doIntersect(ateam_geometry::makeDisk(point, kRobotRadius),
      obstacle_shape));
  }
}
