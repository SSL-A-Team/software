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

using namespace std::string_literals;
using ateam_ai::trajectory_generation::RrtPathPlanner;

bool vectorsAreNear(const Eigen::Vector2d & a, const Eigen::Vector2d & b)
{
  return ateam_common::allCloseDense(a, b);
}

void PrintPath(const RrtPathPlanner::Path & path)
{
  std::ranges::transform(
    path, std::ostream_iterator<std::string>(std::cerr, "\n"), [](const Eigen::Vector2d & p) {
      return std::to_string(p.x()) + ',' + std::to_string(p.y());
    });
}

void ExpectPathDoesNotCollide(
  const RrtPathPlanner::Path & path, const World & world,
  const std::vector<ateam_geometry::AnyShape> & obstacles)
{
  auto robot_footprint = ateam_geometry::makeCircle(ateam_geometry::EigenToPoint(pos), 0.09);
  auto pos_hits_any_obstacle = [&obstacles, &robot_footprint](const Eigen::Vector2d & pos) {
      return std::ranges::any_of(
        obstacles, [&robot_footprint](const auto & obstacle) {
          return ateam_geometry::variantDoIntersect(robot_footprint, obstacle);
        });
    };
  auto pos_hits_any_robot = [&world, &robot_footprint](const Eigen::Vector2d & pos) {
      auto hits_our_robots = std::ranges::any_of(
        world.our_robots, [&pos](const auto & robot) {
          return robot.has_value() && (pos - robot->pos).norm() <= 0.180;
        });
      auto hits_their_robots = std::ranges::any_of(
        world.their_robots, [&pos](const auto & robot) {
          return robot.has_value() && (pos - robot->pos).norm() <= 0.180;
        });
      return hits_our_robots || hits_their_robots;
    };
  if(path.size() < 2) {
    return;  // empty paths cannot collide
  }
  for(auto i = 1u; i < path.size(); ++i) {
    ateam_geometry::Segment path_segment(ateam_geometry::EigenToPoint(path[i-1]), ateam_geometry::EigenToPoint(path[i]));
    // if(std::ranges::any_of(obstacles, [&path_segment]))
    // TODO(barulicm) check full robot width along path
  }
}

void ExpectPathSatisfiesRequest(
  const RrtPathPlanner::Path & path,
  const World & world,
  const std::vector<ateam_geometry::AnyShape> & obstacles,
  const Eigen::Vector2d & start_pos, const Eigen::Vector2d & goal_pos)
{
  // A path should exist
  ASSERT_FALSE(path.empty());

  // The path should connect the desired start and goal positions
  EXPECT_PRED2(vectorsAreNear, path.front(), start_pos) << "Path doesn't start in the right place.";
  EXPECT_PRED2(vectorsAreNear, path.back(), goal_pos) << "Path doesn't end in the right place.";

  // None of the points along the path should hit an obstacle
  // TODO(barulicm) this should check points along edges between path points
  auto pos_hits_any_obstacle = [&obstacles](const Eigen::Vector2d & pos) {
      auto robot_footprint = ateam_geometry::makeCircle(ateam_geometry::EigenToPoint(pos), 0.09);
      return std::ranges::any_of(
        obstacles, [&robot_footprint](const auto & obstacle) {
          return ateam_geometry::variantDoIntersect(robot_footprint, obstacle);
        });
    };
  EXPECT_TRUE(
    std::ranges::none_of(
      path,
      pos_hits_any_obstacle)) << "Path collides with an obstacle!";

  // None of the points along the path should hit a robot
  auto pos_hits_any_robots = [&world](const Eigen::Vector2d & pos) {
      auto hits_robot = [&pos](const std::optional<Robot> & robot) {
          return robot.has_value() && (pos - robot->pos).norm() <= 0.180;
        };
      return std::ranges::any_of(world.our_robots, hits_robot) || std::ranges::any_of(
        world.their_robots, hits_robot);
    };
  EXPECT_TRUE(std::ranges::none_of(path, pos_hits_any_robots));

  if (testing::Test::HasFailure()) {
    std::cerr << "Test failed with path:\n";
    PrintPath(path);
    std::cerr << '\n';
  }
}

TEST(RrtPathPlannerTests, basicTest) {
  RrtPathPlanner planner;

  World world;
  world.field.field_length = 9.0;
  world.field.field_width = 6.0;
  world.field.boundary_width = 0.3;

  Eigen::Vector2d start_pos(0, 0);
  Eigen::Vector2d goal_pos(1, 0);

  auto path = planner.generatePath(world, {}, start_pos, goal_pos);

  ExpectPathSatisfiesRequest(path, world, {}, start_pos, goal_pos);
}

TEST(RrtPathPlannerTests, avoidOneObstacle) {
  RrtPathPlanner planner;

  World world;
  world.field.field_length = 9.0;
  world.field.field_width = 6.0;
  world.field.boundary_width = 0.3;

  Eigen::Vector2d start_pos(0, 0);
  Eigen::Vector2d goal_pos(2, 0);

  std::vector<ateam_geometry::AnyShape> obstacles = {
    ateam_geometry::makeCircle(ateam_geometry::Point(1, 0), 0.09)
  };

  auto path = planner.generatePath(world, obstacles, start_pos, goal_pos);

  ExpectPathSatisfiesRequest(path, world, obstacles, start_pos, goal_pos);
}

// This test is timing out the path planner, taking ~33ms to solve
// TEST(RrtPathPlannerTests, acrossFieldBetweenWalls) {
//   RrtPathPlanner planner;

//   World world;
//   world.field.field_length = 9.0;
//   world.field.field_width = 6.0;
//   world.field.boundary_width = 0.3;

//   /* Test world:
//    * --------------------
//    * |            |    G|
//    * |     |      |     |
//    * |     |      |     |
//    * |     |      |     |
//    * |     |      |     |
//    * |S    |            |
//    * --------------------
//    */

//   Eigen::Vector2d start_pos(-4.5, -3.0);
//   Eigen::Vector2d goal_pos(4.5, 3.0);

//   std::vector<ateam_geometry::AnyShape> obstacles = {
//     ateam_geometry::Segment(ateam_geometry::Point(-1.5, -3.0), ateam_geometry::Point(-1.5, 2.0)),
//     ateam_geometry::Segment(ateam_geometry::Point(1.5, -2.0), ateam_geometry::Point(1.5, 3.0))
//   };

//   auto path = planner.generatePath(world, obstacles, start_pos, goal_pos);

//   ExpectPathSatisfiesRequest(path, world, obstacles, start_pos, goal_pos);
// }

TEST(RrtPathPlannerTests, impossiblePathShouldReturnEmpty)
{
  RrtPathPlanner planner;

  World world;
  world.field.field_length = 9.0;
  world.field.field_width = 6.0;
  world.field.boundary_width = 0.3;

  Eigen::Vector2d start_pos(-4.5, -3.0);
  Eigen::Vector2d goal_pos(4.5, 3.0);

  std::vector<ateam_geometry::AnyShape> obstacles = {
    // Thick wall across the field
    ateam_geometry::Rectangle(ateam_geometry::Point(-1.5, -3.5), ateam_geometry::Point(-1.0, 3.5)),
  };

  auto path = planner.generatePath(world, obstacles, start_pos, goal_pos);

  EXPECT_TRUE(path.empty());
  if (testing::Test::HasFailure()) {
    PrintPath(path);
  }
}

TEST(RrtPathPlannerTests, avoidRobots) {
  RrtPathPlanner planner;

  World world;
  world.field.field_length = 9.0;
  world.field.field_width = 6.0;
  world.field.boundary_width = 0.3;

  world.our_robots = {
    Robot{Eigen::Vector2d(0, 0), {}, {}, {}},
    Robot{Eigen::Vector2d(-0.5, 0.5), {}, {}, {}},
    Robot{Eigen::Vector2d(0.3, -0.3), {}, {}, {}}
  };
  world.their_robots = {
    Robot{Eigen::Vector2d(0.5, -0.5), {}, {}, {}},
    Robot{Eigen::Vector2d(-0.3, 0.3), {}, {}, {}}
  };

  Eigen::Vector2d start_pos(-4.5, -3.0);
  Eigen::Vector2d goal_pos(4.5, 3.0);

  auto path = planner.generatePath(world, {}, start_pos, goal_pos);

  ExpectPathSatisfiesRequest(path, world, {}, start_pos, goal_pos);
}
