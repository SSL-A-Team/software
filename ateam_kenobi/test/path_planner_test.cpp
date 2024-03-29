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

#include "path_planning/path_planner.hpp"
#include "types/world.hpp"
#include "types/robot.hpp"

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
    world.field.boundary_width = 0.01;
    // TODO(barulicm) change these fields when the field geometry fix lands
    world.field.goal_width = 2;
    world.field.goal_depth = 1;
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
    EXPECT_THAT(path, testing::Pointwise(PointsAreNear(), expected_path));
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
  obstacles.push_back(ateam_geometry::makeCircle(ateam_geometry::Point(1, 1), 0.1));
  expected_path = {start, ateam_geometry::Point(1.23762, 0.774008), goal};
  runTest();
}

TEST_F(GetPathTest, PlanAroundTwoObstacles) {
  start = ateam_geometry::Point(0, 0);
  goal = ateam_geometry::Point(2, 2);

  obstacles.push_back(ateam_geometry::makeCircle(ateam_geometry::Point(1, 1), 0.1));
  obstacles.push_back(ateam_geometry::makeCircle(ateam_geometry::Point(1.61881, 1.387004), 0.1));

  expected_path = {start, ateam_geometry::Point(1.88741, 1.23759), goal};
  runTest();
}

TEST_F(GetPathTest, DisallowPlanningOutOfTheField) {
  start = ateam_geometry::Point(0, 0);
  goal = ateam_geometry::Point(10, 10);
  runTest();
}

TEST_F(GetPathTest, DisallowPlanningIntoObstacle) {
  start = ateam_geometry::Point(0, 0);
  goal = ateam_geometry::Point(1, 1);
  obstacles.push_back(ateam_geometry::makeCircle(goal, 1));
  runTest();
}

TEST_F(GetPathTest, DisallowPlanningIntoFriendlyRobots) {
  Robot robot;
  robot.id = 0;
  robot.pos = ateam_geometry::Point(0, 0);
  world.our_robots[0] = robot;

  start = ateam_geometry::Point(2, 2);
  goal = ateam_geometry::Point(0, 0);

  runTest();
}

TEST_F(GetPathTest, DisallowGoalsInOpponentDefenseArea)
{
  start = ateam_geometry::Point(0, 0);
  goal = ateam_geometry::Point(4, 0);
  runTest();
}

TEST_F(GetPathTest, AllowIgnoringStartObstacles)
{
  start = ateam_geometry::Point(0, 0);
  goal = ateam_geometry::Point(1, 1);
  obstacles.push_back(ateam_geometry::makeCircle(start, 0.5));
  expected_path = {start, goal};
  planner_options.ignore_start_obstacle = true;
  runTest();
}

TEST_F(GetPathTest, FailIfNotIgnoringStartObstacles) {
  start = ateam_geometry::Point(0, 0);
  goal = ateam_geometry::Point(1, 1);
  obstacles.push_back(ateam_geometry::makeCircle(start, 0.5));
  planner_options.ignore_start_obstacle = false;
  runTest();
}

TEST_F(GetPathTest, LoopsShouldBeRemoved)
{
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
