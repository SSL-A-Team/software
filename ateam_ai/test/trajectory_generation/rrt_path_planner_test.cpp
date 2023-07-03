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
#include <ateam_common/robot_constants.hpp>
#include <ateam_geometry/ateam_geometry.hpp>

using namespace std::string_literals;
using ateam_ai::trajectory_generation::RrtPathPlanner;
using ateam_ai::trajectory_generation::RrtOptions;

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

void ExpectPositionDoesNotCollide(
  const RrtPathPlanner::Position & position, const World & world,
  const std::vector<ateam_geometry::AnyShape> & obstacles)
{
  const auto robot_footprint = ateam_geometry::makeCircle(
    ateam_geometry::EigenToPoint(
      position), kRobotRadius);

  if (std::ranges::any_of(
      obstacles, [&robot_footprint](const auto & obstacle) {
        return ateam_geometry::variantDoIntersect(robot_footprint, obstacle);
      }))
  {
    FAIL() << "Position (" << position.x() << ", " << position.y() <<
      ") collides with an obstacle.";
  }

  auto pos_hits_robot = [&position](const std::optional<Robot> & robot) {
      return robot.has_value() && (position - robot->pos).norm() <= kRobotDiameter;
    };

  EXPECT_TRUE(
    std::ranges::none_of(
      world.our_robots,
      pos_hits_robot)) << "Position (" << position.x() << ", " << position.y() <<
    ") collides with one of our robots.";

  EXPECT_TRUE(
    std::ranges::none_of(
      world.their_robots,
      pos_hits_robot)) << "Position (" << position.x() << ", " << position.y() <<
    ") collides with one of their robots.";
}


void ExpectPathDoesNotCollide(
  const RrtPathPlanner::Path & path, const World & world,
  const std::vector<ateam_geometry::AnyShape> & obstacles)
{
  const auto kCollisionDetectStepSize = 0.1;

  if (path.size() < 2) {
    return;  // empty paths cannot collide
  }
  for (auto i = 1u; i < path.size(); ++i) {
    const Eigen::Vector2d segment_start = path[i - 1];
    const Eigen::Vector2d segment_end = path[i];
    const Eigen::Vector2d step_vector = (segment_end - segment_start).normalized() *
      kCollisionDetectStepSize;
    const int step_count = (segment_end - segment_start).norm() / kCollisionDetectStepSize;
    for (auto step = 0; step < step_count; ++step) {
      const Eigen::Vector2d position = segment_start + (step * step_vector);
      ExpectPositionDoesNotCollide(position, world, obstacles);
    }
    // check final position in case the segment length did not evenly divide into steps
    ExpectPositionDoesNotCollide(path.back(), world, obstacles);
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

  ExpectPathDoesNotCollide(path, world, obstacles);

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

TEST(RrtPathPlannerTests, acrossFieldBetweenWalls) {
  RrtPathPlanner planner;

  World world;
  world.field.field_length = 9.0;
  world.field.field_width = 6.0;
  world.field.boundary_width = 0.3;

  /* Test world:
   * --------------------
   * |            |    G|
   * |     |      |     |
   * |     |      |     |
   * |     |      |     |
   * |     |      |     |
   * |S    |            |
   * --------------------
   */

  Eigen::Vector2d start_pos(-4.5, -3.0);
  Eigen::Vector2d goal_pos(4.5, 3.0);

  std::vector<ateam_geometry::AnyShape> obstacles = {
    ateam_geometry::Segment(ateam_geometry::Point(-1.5, -3.0), ateam_geometry::Point(-1.5, 2.0)),
    ateam_geometry::Segment(ateam_geometry::Point(1.5, -2.0), ateam_geometry::Point(1.5, 3.0))
  };

  RrtOptions options;
  /* This is longer than we normally want the planner to run. I'm keeping this test in because
   * if something makes this take longer, it likely hurts the overal performance of our planner.
   */
  options.search_time_limit = 0.05;
  options.simplification_time_limit = 0.03;
  auto path = planner.generatePath(world, obstacles, start_pos, goal_pos, options);

  ExpectPathSatisfiesRequest(path, world, obstacles, start_pos, goal_pos);
}

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
