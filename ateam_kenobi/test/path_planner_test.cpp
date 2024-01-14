// Copyright 2023 A Team
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

#include "path_planning/path_planner.hpp"
#include "types/world.hpp"
#include "types/robot.hpp"

#include <ateam_geometry/ateam_geometry.hpp>

namespace ateam_kenobi
{

class GetPathTest : public ::testing::Test
{
protected:
  path_planning::PathPlanner path_planner;
  World world;
  path_planning::PlannerOptions planner_options;
  std::vector<ateam_geometry::AnyShape> obstacles;
  std::chrono::time_point<std::chrono::steady_clock> start_time;
  std::chrono::duration<double> allowed_extra_time = std::chrono::milliseconds(100);
  path_planning::PathPlanner::Path empty_path = {};

  GetPathTest() {}
  virtual ~GetPathTest() {}

  void SetUp() override
  {
    world.field.field_length = 9;
    world.field.field_width = 6;
    world.field.boundary_width = 0.01;
    world.field.goal_width = 2;
    world.field.goal_depth = 1;
    world.ball.pos = ateam_geometry::Point(-4.5, -3.0);
    start_time = std::chrono::steady_clock::now();
  }
  virtual void TearDown()
  {
    obstacles.erase(obstacles.begin(), obstacles.end());
  }
};

/* Test whether we can make a basic path between two points
 * without any weird stuff happening (adding more than 1 point or splitting).
 */
TEST_F(GetPathTest, StraightPath) {
  const auto start = ateam_geometry::Point(0, 0);
  const auto end = ateam_geometry::Point(1, 1);
  path_planning::PathPlanner::Path path = {start, end};
  auto planner_path = path_planner.getPath(
    start, end, world, obstacles, planner_options);
  const auto end_time = std::chrono::steady_clock::now();
  EXPECT_NE(planner_path, empty_path);
  EXPECT_EQ(planner_path, path);
  EXPECT_EQ(path.size(), 2U);
  EXPECT_LT(
    end_time - start_time,
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(
        planner_options.search_time_limit)) + allowed_extra_time);
}

/* Test whether we can make a path around an obstacle without
 * any weird stuff happening (going through the obstacle, not making a path).
 */
TEST_F(GetPathTest, PathWithSingleObstacle) {
  const auto start = ateam_geometry::Point(0, 0);
  const auto end = ateam_geometry::Point(2, 2);
  path_planning::PathPlanner::Path short_path = {start, end};
  const auto obstacle = ateam_geometry::makeCircle(ateam_geometry::Point(1, 1), 0.1);
  obstacles.push_back(obstacle);
  auto path = path_planner.getPath(
    start, end, world, obstacles, planner_options);
  const auto end_time = std::chrono::steady_clock::now();
  EXPECT_NE(path, empty_path);
  EXPECT_NE(path, short_path);
  EXPECT_GT(path.size(), 2U);
  EXPECT_LT(
    end_time - start_time,
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(
        planner_options.search_time_limit)) + allowed_extra_time);

  // std::cout << "Found path:\n";
  // for (const auto p : path) {
  //   std::cout << '\t' << p << '\n';
  // }
  // FAIL();
}

/* Test whether we can make a path around multiple obstacles without any weird stuff
 * happening (not finishing in time, going through the obstacles).
 */
TEST_F(GetPathTest, PathWithMultipleObstacles) {
  const auto start = ateam_geometry::Point(0, 0);
  const auto end = ateam_geometry::Point(2, 2);
  path_planning::PathPlanner::Path short_path = {start, end};
  const auto obstacle_1 = ateam_geometry::makeCircle(ateam_geometry::Point(1, 1), 0.1);
  obstacles.push_back(obstacle_1);
  const auto obstacle_2 = ateam_geometry::makeCircle(ateam_geometry::Point(1.61881, 1.387004), 0.1);
  obstacles.push_back(obstacle_2);
  auto path = path_planner.getPath(
    start, end, world, obstacles, planner_options);
  const auto end_time = std::chrono::steady_clock::now();
  EXPECT_NE(path, empty_path);
  EXPECT_NE(path, short_path);
  EXPECT_GT(path.size(), 2U);
  EXPECT_LT(
    end_time - start_time,
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(
        planner_options.search_time_limit)) + allowed_extra_time);

  // std::cout << "Found path:\n";
  // for (const auto p : path) {
  //   std::cout << '\t' << p << '\n';
  // }
  // FAIL();
}

/* Test whether an out of bounds point correctly is identified as invalid.
 * In this case, we should get an empty path from the planner.
 */
TEST_F(GetPathTest, OutOfBounds) {
  const auto start = ateam_geometry::Point(0, 0);
  const auto invalid_point = ateam_geometry::Point(10, 10);
  auto path = path_planner.getPath(
    start, invalid_point, world, obstacles, planner_options);
  const auto end_time = std::chrono::steady_clock::now();
  EXPECT_EQ(path, empty_path);
  EXPECT_EQ(path.size(), 0U);
  EXPECT_LT(
    end_time - start_time,
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(
        planner_options.search_time_limit)) + allowed_extra_time);
}

/* Test whether a point inside of an obstacle correctly is identified as invalid.
 * In this case, we should get an empty path from the planner.
 */
TEST_F(GetPathTest, InObstacle) {
  const auto start = ateam_geometry::Point(0, 0);
  const auto obstacle = ateam_geometry::makeCircle(ateam_geometry::Point(1, 1), 1);
  const auto invalid_point = ateam_geometry::Point(1, 1);
  obstacles.push_back(obstacle);
  auto path = path_planner.getPath(
    start, invalid_point, world, obstacles, planner_options);
  const auto end_time = std::chrono::steady_clock::now();
  EXPECT_EQ(path, empty_path);
  EXPECT_EQ(path.size(), 0U);
  EXPECT_LT(
    end_time - start_time,
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(
        planner_options.search_time_limit)) + allowed_extra_time);
}

/* Test whether a robot in the world correctly gets added to the list of obstacles.
 */
TEST_F(GetPathTest, CreateObstaclesFromRobots) {
  auto robot = Robot();
  robot.id = 0;
  robot.pos = ateam_geometry::Point(0, 0);
  world.our_robots[0] = robot;
  const auto start = ateam_geometry::Point(2, 2);
  const auto invalid_point = ateam_geometry::Point(0, 0);
  auto path = path_planner.getPath(
    start, invalid_point, world, obstacles, planner_options);
  const auto end_time = std::chrono::steady_clock::now();
  EXPECT_EQ(path, empty_path);
  EXPECT_EQ(path.size(), 0U);
  EXPECT_LT(
    end_time - start_time,
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(
        planner_options.search_time_limit)) + allowed_extra_time);
}

/* Test that path planner correctly rejects goals in opponent defense area
 */
TEST_F(GetPathTest, DisallowGoalsInOpponentDefenseArea)
{
  const auto start = ateam_geometry::Point(0,0);
  const auto goal = ateam_geometry::Point(4, 0);
  const auto path = path_planner.getPath(start, goal, world, obstacles, planner_options);
  const auto end_time = std::chrono::steady_clock::now();
  EXPECT_EQ(path, empty_path);
  EXPECT_EQ(path.size(), 0U);
  EXPECT_LT(
    end_time - start_time,
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(
        planner_options.search_time_limit)) + allowed_extra_time);
}

}  // namespace ateam_kenobi
