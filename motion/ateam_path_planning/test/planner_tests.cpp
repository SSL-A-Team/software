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
#include "utils/path_ends_at.hpp"
#include "utils/path_starts_at.hpp"
#include "utils/path_avoids_obstacles.hpp"
#include "utils/paths_collide.hpp"

using ateam_path_planning::Planner;

using ::testing::Eq;
using ::testing::Field;
using ::testing::Not;
using ::testing::Optional;
using ::testing::SizeIs;


#define SegmentCount(c) Field(&ateam_path_planning::TrajectorySpline::segments, SizeIs(c))

void PrintPathsOnFailure(
  const std::array<std::optional<ateam_path_planning::TrajectorySpline>,
  16> & paths)
{
  if(!::testing::Test::HasFailure()) {
    return;
  }
  for(const auto & path : paths) {
    if(!path.has_value()) {
      continue;
    }
    std::cerr << "[\n";
    const auto points =
      path->ToPoints(ateam_path_planning::PlannerOptions{}.collision_check_resolution);
    for(auto i = 0; i < points.size(); ++i) {
      std::cerr << '(' << points[i].x() << ", " << points[i].y() << "),\n";
    }
    std::cerr << "],\n";
  }
}

// TEST(Planner, AllBotsNoTargets) {
//   Planner planner;

//   const auto paths = planner.PlanPathsForAllBots({}, {}, {}, {}, {});

//   EXPECT_THAT(paths, ::testing::Each(::testing::Eq(std::nullopt)));
// }

// TEST(Planner, OneBotNoObstacles) {
//   Planner planner;

//   std::array<std::optional<ateam_path_planning::Pose>, 16> targets;
//   targets.fill(std::nullopt);
//   targets[0] = ateam_path_planning::Pose{ateam_geometry::Point(1.0, 1.0), 0.0};

//   std::array<uint8_t, 16> priorities;
//   priorities.fill(0);

//   ateam_game_state::World world;
//   world.our_robots[0].id = 0;
//   world.our_robots[0].pos = ateam_geometry::Point(0.0, 0.0);
//   world.our_robots[0].theta = 0.0;
//   world.our_robots[0].vel = ateam_geometry::Vector(0.0, 0.0);

//   const auto paths = planner.PlanPathsForAllBots(targets, priorities, world, {}, {});

//   EXPECT_THAT(paths[0],
//     Optional(PathStartsAt(ateam_path_planning::Pose{ateam_geometry::Point(0.0, 0.0), 0.0})));
//   EXPECT_THAT(paths[0],
//     Optional(PathEndsAt(ateam_path_planning::Pose{ateam_geometry::Point(1.0, 1.0), 0.0})));
//   EXPECT_THAT(paths[0], Optional(SegmentCount(1)));

//   for (size_t i = 1; i < paths.size(); ++i) {
//     EXPECT_THAT(paths[i], Eq(std::nullopt));
//   }

//   PrintPathsOnFailure(paths);
// }

// TEST(Planner, OneBotOneObstacle) {
//   Planner planner;

//   std::array<std::optional<ateam_path_planning::Pose>, 16> targets;
//   targets.fill(std::nullopt);
//   targets[0] = ateam_path_planning::Pose{ateam_geometry::Point(1.0, 1.0), 0.0};

//   std::array<uint8_t, 16> priorities;
//   priorities.fill(0);

//   ateam_game_state::World world;
//   world.our_robots[0].id = 0;
//   world.our_robots[0].pos = ateam_geometry::Point(0.0, 0.0);
//   world.our_robots[0].theta = 0.0;
//   world.our_robots[0].vel = ateam_geometry::Vector(0.0, 0.0);

//   const auto obstacle_shape = ateam_geometry::makeDisk(ateam_geometry::Point(0.5, 0.5), 0.1);

//   std::vector<ateam_path_planning::Obstacle> global_obstacles = {
//     ateam_path_planning::Obstacle{obstacle_shape, {}}
//   };

//   const auto paths = planner.PlanPathsForAllBots(targets, priorities, world, global_obstacles, {});

//   EXPECT_THAT(paths[0],
//     Optional(PathStartsAt(ateam_path_planning::Pose{ateam_geometry::Point(0.0, 0.0), 0.0})));
//   EXPECT_THAT(paths[0],
//     Optional(PathEndsAt(ateam_path_planning::Pose{ateam_geometry::Point(1.0, 1.0), 0.0})));
//   EXPECT_THAT(paths[0], Optional(SegmentCount(2)));
//   EXPECT_THAT(paths[0], Optional(PathAvoidsObstacles(global_obstacles)));
//   for (size_t i = 1; i < paths.size(); ++i) {
//     EXPECT_THAT(paths[i], Eq(std::nullopt));
//   }

//   PrintPathsOnFailure(paths);
// }

// TEST(Planner, OneBotMovingObstacle) {
//   Planner planner;

//   std::array<std::optional<ateam_path_planning::Pose>, 16> targets;
//   targets.fill(std::nullopt);
//   targets[0] = ateam_path_planning::Pose{ateam_geometry::Point(2.0, 0.0), 0.0};

//   std::array<uint8_t, 16> priorities;
//   priorities.fill(0);

//   ateam_game_state::World world;
//   world.our_robots[0].id = 0;
//   world.our_robots[0].pos = ateam_geometry::Point(0.0, 0.0);
//   world.our_robots[0].theta = 0.0;
//   world.our_robots[0].vel = ateam_geometry::Vector(0.0, 0.0);

//   const auto obstacle_shape = ateam_geometry::makeDisk(ateam_geometry::Point(1.0, 1.0), 0.1);

//   std::vector<ateam_path_planning::Obstacle> global_obstacles = {
//     ateam_path_planning::Obstacle{obstacle_shape, ateam_geometry::Vector(0.0, -1.0)}
//   };

//   const auto paths = planner.PlanPathsForAllBots(targets, priorities, world, global_obstacles, {});

//   EXPECT_THAT(paths[0],
//     Optional(PathStartsAt(ateam_path_planning::Pose{ateam_geometry::Point(0.0, 0.0), 0.0})));
//   EXPECT_THAT(paths[0],
//     Optional(PathEndsAt(ateam_path_planning::Pose{ateam_geometry::Point(2.0, 0.0), 0.0})));
//   EXPECT_THAT(paths[0], Optional(SegmentCount(2)));
//   EXPECT_THAT(paths[0], Optional(PathAvoidsObstacles(global_obstacles)));
//   for (size_t i = 1; i < paths.size(); ++i) {
//     EXPECT_THAT(paths[i], Eq(std::nullopt));
//   }

//   PrintPathsOnFailure(paths);
// }

TEST(Planner, AllBotsCrossNoObstacles) {
  Planner planner;

  std::array<std::optional<ateam_path_planning::Pose>, 16> targets;
  std::generate(targets.begin(), targets.end(), [n = 15]() mutable {
      return ateam_path_planning::Pose{ateam_geometry::Point(3.0, n-- * 0.5), 0.0};
  });

  std::array<uint8_t, 16> priorities;
  priorities.fill(0);

  ateam_game_state::World world;
  std::generate(world.our_robots.begin(), world.our_robots.end(), [n = 0]() mutable {
      ateam_game_state::Robot robot;
      robot.id = n;
      robot.pos = ateam_geometry::Point(0.0, n * 0.5);
      robot.theta = 0.0;
      robot.vel = ateam_geometry::Vector(0.0, 0.0);
      ++n;
      return robot;
  });

  const auto paths = planner.PlanPathsForAllBots(targets, priorities, world, {}, {});

  for (size_t i = 0; i < paths.size(); ++i) {
    EXPECT_THAT(paths[i],
      Optional(PathStartsAt(ateam_path_planning::Pose{ateam_geometry::Point(0.0, i * 0.5),
        0.0})));
    EXPECT_THAT(paths[i],
      Optional(PathEndsAt(ateam_path_planning::Pose{ateam_geometry::Point(3.0, (15 - i) * 0.5),
        0.0})));
  }

  EXPECT_THAT(paths, Not(PathsCollide()));

  EXPECT_TRUE(false);

  PrintPathsOnFailure(paths);
}

// TEST(Planner, TwoBotOneObstacle) {
//   Planner planner;

//   std::array<std::optional<ateam_path_planning::Pose>, 16> targets;
//   targets.fill(std::nullopt);
//   targets[0] = ateam_path_planning::Pose{ateam_geometry::Point(1.0, 0.2), 0.0};
//   targets[1] = ateam_path_planning::Pose{ateam_geometry::Point(1.0, -0.2), 0.0};

//   std::array<uint8_t, 16> priorities;
//   priorities.fill(0);

//   ateam_game_state::World world;
//   world.our_robots[0].id = 0;
//   world.our_robots[0].pos = ateam_geometry::Point(-1.0, 0.2);
//   world.our_robots[0].theta = 0.0;
//   world.our_robots[0].vel = ateam_geometry::Vector(0.0, 0.0);
//   world.our_robots[1].id = 1;
//   world.our_robots[1].pos = ateam_geometry::Point(-1.0, -0.2);
//   world.our_robots[1].theta = 0.0;
//   world.our_robots[1].vel = ateam_geometry::Vector(0.0, 0.0);

//   const auto obstacle_shape = ateam_geometry::makeDisk(ateam_geometry::Point(0.0, 0.0), 0.2);

//   std::vector<ateam_path_planning::Obstacle> global_obstacles = {
//     ateam_path_planning::Obstacle{obstacle_shape, {}}
//   };

//   const auto paths = planner.PlanPathsForAllBots(targets, priorities, world, global_obstacles, {});

//   EXPECT_THAT(paths[0],
//     Optional(PathStartsAt(ateam_path_planning::Pose{ateam_geometry::Point(-1.0, 0.2), 0.0})));
//   EXPECT_THAT(paths[0],
//     Optional(PathEndsAt(ateam_path_planning::Pose{ateam_geometry::Point(1.0, 0.2), 0.0})));
//   EXPECT_THAT(paths[0], Optional(PathAvoidsObstacles(global_obstacles)));
//   EXPECT_THAT(paths[1],
//     Optional(PathStartsAt(ateam_path_planning::Pose{ateam_geometry::Point(-1.0, -0.2), 0.0})));
//   EXPECT_THAT(paths[1],
//     Optional(PathEndsAt(ateam_path_planning::Pose{ateam_geometry::Point(1.0, -0.2), 0.0})));
//   EXPECT_THAT(paths[1], Optional(PathAvoidsObstacles(global_obstacles)));
//   for (size_t i = 2; i < paths.size(); ++i) {
//     EXPECT_THAT(paths[i], Eq(std::nullopt));
//   }

//   PrintPathsOnFailure(paths);
// }
