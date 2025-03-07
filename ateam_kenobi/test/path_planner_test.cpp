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

#include <chrono>
#include <random>

#include "core/path_planning/path_planner.hpp"
#include "core/types/world.hpp"
#include "core/types/robot.hpp"

#include <ateam_geometry/ateam_geometry.hpp>
#include <ateam_geometry_testing/testing_utils.hpp>

using namespace ateam_kenobi;  // NOLINT(build/namespaces)
using namespace ateam_kenobi::path_planning;  // NOLINT(build/namespaces)

class GetPathTest : public ::testing::Test
{
protected:
  const std::chrono::milliseconds kAllowedExtraTime{10};
  PathPlanner path_planner;
  World world;
  PlannerOptions planner_options;
  std::vector<ateam_geometry::AnyShape> obstacles;
  std::chrono::nanoseconds execution_time;
  ateam_geometry::Point start;
  ateam_geometry::Point goal;
  PathPlanner::Path expected_path;

  void SetUp() override
  {
    world = {};
    world.field.field_length = 9;
    world.field.field_width = 6;
    world.field.boundary_width = 0.3;
    world.field.goal_width = 1;
    world.field.goal_depth = 0.1;
    const auto defense_area_width = 2.0;
    const auto defense_area_depth = 1.0;
    world.field.defense_area_width = defense_area_width;
    world.field.defense_area_depth = defense_area_depth;
    const auto half_field_length = world.field.field_length / 2.0;
    world.field.ours.defense_area_corners = {
      ateam_geometry::Point(half_field_length, -defense_area_width / 2.0),
      ateam_geometry::Point(half_field_length, defense_area_width / 2.0),
      ateam_geometry::Point(half_field_length - defense_area_depth, defense_area_width / 2.0),
      ateam_geometry::Point(half_field_length - defense_area_depth, -defense_area_width / 2.0),
    };
    world.field.theirs.defense_area_corners = {
      ateam_geometry::Point(-half_field_length, -defense_area_width / 2.0),
      ateam_geometry::Point(-half_field_length, defense_area_width / 2.0),
      ateam_geometry::Point(-half_field_length + defense_area_depth, defense_area_width / 2.0),
      ateam_geometry::Point(-half_field_length + defense_area_depth, -defense_area_width / 2.0),
    };
    world.ball.pos = ateam_geometry::Point(-4.5, -3.0);
    planner_options = {};
    obstacles.clear();
    execution_time = std::chrono::nanoseconds(0);
    start = {};
    goal = {};
    expected_path.clear();
  }

  PathPlanner::Path getPath()
  {
    const auto start_time = std::chrono::steady_clock::now();
    const auto path = path_planner.getPath(start, goal, world, obstacles, planner_options);
    const auto end_time = std::chrono::steady_clock::now();
    execution_time = end_time - start_time;
    return path;
  }

  void expectExecutionTimeWithinLimits()
  {
    EXPECT_LE(
      execution_time, std::chrono::duration<double>(
        planner_options.search_time_limit) + kAllowedExtraTime) <<
      "PathPlanner took too long to find its path.";
  }

  void printPath(const ateam_kenobi::path_planning::PathPlanner::Path & path)
  {
    std::cout << "Path:\n";
    for (const auto & p : path) {
      std::cout << p.x() << ", " << p.y() << '\n';
    }
  }

  void runTest()
  {
    const auto path = getPath();
    expectExecutionTimeWithinLimits();
    const auto millimeter = 1e-3;
    EXPECT_THAT(path, testing::Pointwise(PointsAreNear(millimeter), expected_path));
    if (::testing::Test::HasFailure()) {
      printPath(path);
    }
  }
};

TEST_F(GetPathTest, PlanWithoutObstacles) {
  start = ateam_geometry::Point(0, 0);
  goal = ateam_geometry::Point(1, 1);
  planner_options.use_default_obstacles = false;
  expected_path = {start, goal};
  runTest();
}

TEST_F(GetPathTest, PlanAroundOneObstacle) {
  start = ateam_geometry::Point(0, 0);
  goal = ateam_geometry::Point(2, 2);
  obstacles.push_back(ateam_geometry::makeDisk(ateam_geometry::Point(1, 1), 0.1));
  expected_path = {start, ateam_geometry::Point(1.23762, 0.774008), goal};
  runTest();
}

TEST_F(GetPathTest, PlanAroundTwoObstacles) {
  start = ateam_geometry::Point(0, 0);
  goal = ateam_geometry::Point(2, 2);

  obstacles.push_back(ateam_geometry::makeDisk(ateam_geometry::Point(1, 1), 0.1));
  obstacles.push_back(ateam_geometry::makeDisk(ateam_geometry::Point(1.61881, 1.387004), 0.1));

  expected_path = {start, ateam_geometry::Point(1.86984, 1.19029), goal};
  runTest();
}

TEST_F(GetPathTest, DisallowPlanningOutOfTheField) {
  start = ateam_geometry::Point(0, 0);
  goal = ateam_geometry::Point(10, 10);
  // Planner returns path to closest valid point to goal along start-goal line
  expected_path = {start, ateam_geometry::Point(3.21177, 3.21177)};
  runTest();
}

TEST_F(GetPathTest, DisallowPlanningIntoObstacle) {
  start = ateam_geometry::Point(0, 0);
  goal = ateam_geometry::Point(1, 1);
  obstacles.push_back(ateam_geometry::makeDisk(goal, 1));
  // Planner returns path to closest valid point to goal along start-goal line
  expected_path = {start, ateam_geometry::Point(0.151472, 0.151472)};
  runTest();
}

TEST_F(GetPathTest, DisallowPlanningIntoFriendlyRobots) {
  world.our_robots[0].id = 0;
  world.our_robots[0].visible = true;
  world.our_robots[0].pos = ateam_geometry::Point(0, 0);

  start = ateam_geometry::Point(2, 2);
  goal = ateam_geometry::Point(0, 0);

  // Planner returns path to closest valid point to goal along start-goal line
  expected_path = {start, ateam_geometry::Point(0.176777, 0.176777)};
  runTest();
}

TEST_F(GetPathTest, DisallowGoalsInOpponentDefenseArea)
{
  start = ateam_geometry::Point(0, 0);
  goal = ateam_geometry::Point(4, 0);
  // Planner returns path to closest valid point to goal along start-goal line
  expected_path = {start, ateam_geometry::Point(3.3, 0.0)};
  runTest();
}

TEST_F(GetPathTest, AllowIgnoringStartObstacles)
{
  start = ateam_geometry::Point(0, 0);
  goal = ateam_geometry::Point(1, 1);
  obstacles.push_back(ateam_geometry::makeDisk(start, 0.5));
  expected_path = {start, goal};
  planner_options.ignore_start_obstacle = true;
  runTest();
}

TEST_F(GetPathTest, FailIfNotIgnoringStartObstacles) {
  start = ateam_geometry::Point(0, 0);
  goal = ateam_geometry::Point(1, 1);
  obstacles.push_back(ateam_geometry::makeDisk(start, 0.5));
  planner_options.ignore_start_obstacle = false;
  runTest();
}

TEST_F(GetPathTest, DISABLED_LoopsShouldBeRemoved)
{
  // We broke this test at some point during quals prep.
  /* This scenario produces a raw path with loops (the path crosses over itself). The test ensures
   * the loop removal function works.
   */
  start = ateam_geometry::Point(0.0465311, 0.244153);
  goal = ateam_geometry::Point(0.246488, 0.0577052);
  world.ball.pos = ateam_geometry::Point(0.296055, 0.299946);
  Robot opponent;
  opponent.id = 0;
  opponent.pos = ateam_geometry::Point(0.00493251, 0.00675831);
  world.their_robots[0] = opponent;
  expected_path = {
    start,
    ateam_geometry::Point(0.0808856, 0.338844),
    goal
  };
  runTest();
}
