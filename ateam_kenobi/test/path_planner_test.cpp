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

#include <chrono>

#include "path_planning/path_planner.hpp"
#include "types/world.hpp"
#include "types/robot.hpp"

#include <ateam_geometry/ateam_geometry.hpp>

namespace ateam_kenobi
{
/*
    *   Fixture for path test setup
    */
class GetPathTest : public ::testing::Test
{
protected:
  path_planning::PathPlanner path_planner;
  World world;
  path_planning::PlannerOptions planner_options;
  std::vector<ateam_geometry::AnyShape> obstacles;
  std::chrono::time_point<std::chrono::steady_clock> start_time;
  std::chrono::duration<double> allowed_extra_time = std::chrono::milliseconds(100);

  GetPathTest() {}
  virtual ~GetPathTest() {}

  /**
           * Setup function for path planning tests. Does the following:
           * Set mock field message based on definitions in section 2 of SSL rule book:
           * https://robocup-ssl.github.io/ssl-rules/sslrules.html#_field_setup
           */
  void SetUp() override
  {
    world.field.field_length = 9;
    world.field.field_width = 6;
    world.field.boundary_width = 0.01;
    world.ball.pos = ateam_geometry::Point(50, 50);
    start_time = std::chrono::steady_clock::now();
  }
  virtual void TearDown()
  {
    obstacles.erase(obstacles.begin(), obstacles.end());
  }
};

/*
    *   Test whether we can make a basic path between two points
    *   without any weird stuff happening (adding more than 1 point or splitting).
    */
TEST_F(GetPathTest, StraightPath) {
  const auto start = ateam_geometry::Point(0, 0);
  const auto end = ateam_geometry::Point(1, 1);
  path_planning::PathPlanner::Path path = {start, end};
  path_planning::PathPlanner::Path empty_path = {};
  auto planner_path = path_planner.getPath(
    start, end, world, obstacles, planner_options);
  const auto end_time = std::chrono::steady_clock::now();
  EXPECT_NE(planner_path, empty_path);
  EXPECT_EQ(planner_path, path);
  //EXPECT_EQ(path.size(), 2);
  //EXPECT_LT(end_time - start_time, std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(planner_options.search_time_limit)) + allowed_extra_time);
}

// /*
//     *   Test whether we can make a path around an obstacle without
//     *   any weird stuff happening (going through the obstacle, not making a path).
//     */
// TEST_F(GetPathTest, PathWithSingleObstacle) {
//   const auto start = ateam_geometry::Point(0, 0);
//   const auto end = ateam_geometry::Point(3, 3);
//   const auto obstacle = ateam_geometry::makeCircle(ateam_geometry::Point(1, 1), 0.1);
//   obstacles.push_back(obstacle);
//   auto path = path_planner.getPath(
//     start, end, world, obstacles, planner_options);
//   const auto end_time = std::chrono::duration_cast<std::chrono::duration<double>>(
//     std::chrono::steady_clock::now());
//   EXPECT_NE(path, {});
//   EXPECT_NE(path, {start, end});
//   EXPECT_GT(path.size(), 2);
//   EXPECT_LT(end_time - start_time, planner_options.search_time_limit + allowed_extra_time);
// }

// /*
//     *   Test whether we can make a path around multiple obstacles without any weird stuff happening
//     *   (not finishing in time, going through the obstacles).
//     */
// TEST_F(GetPathTest, PathWithMultipleObstacles) {
//   const auto start = ateam_geometry::Point(0, 0);
//   const auto end = ateam_geometry::Point(3, 3);
//   const auto obstacle_1 = ateam_geometry::makeCircle(ateam_geometry::Point(1, 1), 0.1);
//   obstacles.push_back(obstacle_1);
//   const auto obstacle_2 = ateam_geometry::makeCircle(ateam_geometry::Point(1, 2), 0.1);
//   obstacles.push_back(obstacle_2);
//   auto path = path_planner.getPath(
//     start, end, world, obstacles, planner_options);
//   const auto end_time = std::chrono::duration_cast<std::chrono::duration<double>>(
//     std::chrono::steady_clock::now());
//   EXPECT_NE(path, {});
//   EXPECT_NE(path, {start, end});
//   EXPECT_GT(path.size(), 2);
//   EXPECT_LT(end_time - start_time, planner_options.search_time_limit + allowed_extra_time);
// }

// /*
//     *   Test whether an out of bounds point correctly is identified as invalid.
//     */
// TEST_F(GetPathTest, OutOfBounds) {
//   const auto invalid_point = ateam_geometry::Point(10, 10);
//   bool is_valid = path_planner.isStateValid(invalid_point, world, obstacles, planner_options);
//   EXPECT_FALSE(is_valid);
// }

// /*
//     *   Test whether a point inside of an obstacle correctly is identified as invalid.
//     */
// TEST_F(GetPathTest, InObstacle) {
//   const auto obstacle = ateam_geometry::makeCircle(ateam_geometry::Point(1, 1), 1);
//   obstacles.push_back(obstacle);
//   const auto invalid_point = ateam_geometry::Point(10, 10);
//   bool is_valid = path_planner.isStateValid(invalid_point, world, obstacles, planner_options);
//   EXPECT_FALSE(is_valid);
// }

// /*
//     *   Test whether a robot in the world correctly gets added to the list of obstacles.
//     */
// TEST_F(GetPathTest, CreateObstaclesFromRobots) {
//   const auto robot = Robot();
//   robot.id = 0;
//   robot.pos = ateam_geometry::Point(0, 0);
//   robot.vel = 0;
//   world.our_robots[0].value() = robot;
//   const auto start_pos = ateam_geometry::Point(2, 2);
//   path_planner.addRobotsToObstacles(world, start_pos, obstacles);
//   EXPECT_FALSE(obstacles.empty());
//   EXPECT_EQ(1, obstacles.size());
//   EXPECT_EQ(ateam_geometry::makeCircle(robot.value().pos, 0.09), obstacles.front());
// }

// /*
//     *   Test whether we correctly add unallowed play areas to the list of default obstacles.
//     */
// TEST_F(GetPathTest, GetDefaultObstacles) {
//   path_planner.addDefaultObstacles(world, obstacles);
//   EXPECT_EQ(2, obstacles.size());
//   EXPECT_FALSE(obstacles.empty());
//   EXPECT_EQ(
//     ateam_geometry::Rectangle(
//       ateam_geometry::Point(-world.field.field_length / 2, world.field.goal_width),
//       ateam_geometry::Point(
//         -1 * (world.field.field_length / 2) + world.field.goal_width,
//         -world.field.goal_width)
//     ), obstacles.front());
//   EXPECT_EQ(
//     ateam_geometry::Rectangle(
//       ateam_geometry::Point((world.field.field_length / 2), world.field.goal_width),
//       ateam_geometry::Point(
//         (world.field.field_length / 2) + world.field.goal_width,
//         -world.field.goal_width)
//     ), obstacles.back());
// }
}  // namespace ateam_kenobi
