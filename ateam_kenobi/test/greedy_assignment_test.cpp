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
#include "robot_assignment.hpp"


using namespace ateam_kenobi;  // NOLINT(build/namespaces)
using namespace ateam_kenobi::robot_assignment;  // NOLINT(build/namespaces)

TEST(GreedyAssignmentTest, EmptyRobots)
{
  std::vector<Robot> robots;
  std::vector<ateam_geometry::Point> goals = {
    ateam_geometry::Point(0, 0)
  };
  const auto assignments = greedyAssignment(robots, goals);
  EXPECT_TRUE(assignments.empty());
}

TEST(GreedyAssignmentTest, EmptyGoals)
{
  std::vector<Robot> robots = {
    {}
  };
  std::vector<ateam_geometry::Point> goals;
  const auto assignments = greedyAssignment(robots, goals);
  EXPECT_TRUE(assignments.empty());
}

TEST(GreedyAssignmentTest, OneRobotOneGoal)
{
  std::vector<Robot> robots {
    {1, ateam_geometry::Point(1, 2)}
  };
  std::vector<ateam_geometry::Point> goals = {
    ateam_geometry::Point(3, 4)
  };
  const auto assignments = greedyAssignment(robots, goals);
  EXPECT_THAT(assignments, testing::UnorderedElementsAre(testing::Pair(1, 0)));
}

TEST(GreedyAssignmentTest, TwoRobotsOneGoal)
{
  std::vector<Robot> robots {
    {1, ateam_geometry::Point(1, 2)},
    {2, ateam_geometry::Point(3, 4)}
  };
  std::vector<ateam_geometry::Point> goals = {
    ateam_geometry::Point(3, 4)
  };
  const auto assignments = greedyAssignment(robots, goals);
  EXPECT_THAT(assignments, testing::UnorderedElementsAre(testing::Pair(2, 0)));
}

TEST(GreedyAssignmentTest, TwoRobotsTwoGoals)
{
  std::vector<Robot> robots {
    {1, ateam_geometry::Point(1, 2)},
    {2, ateam_geometry::Point(3, 4)}
  };
  std::vector<ateam_geometry::Point> goals = {
    ateam_geometry::Point(0, 0),
    ateam_geometry::Point(3, 4)
  };
  const auto assignments = greedyAssignment(robots, goals);
  EXPECT_THAT(assignments, testing::UnorderedElementsAre(testing::Pair(1, 0), testing::Pair(2, 1)));
}

TEST(GreedyAssignmentTest, TwoRobotsSameDistance)
{
  std::vector<Robot> robots {
    {1, ateam_geometry::Point(0, 1)},
    {2, ateam_geometry::Point(0, -1)}
  };
  std::vector<ateam_geometry::Point> goals = {
    ateam_geometry::Point(1, 0),
    ateam_geometry::Point(-1, 0)
  };
  const auto assignments = greedyAssignment(robots, goals);
  EXPECT_THAT(assignments, testing::UnorderedElementsAre(testing::Pair(1, 0), testing::Pair(2, 1)));
}
