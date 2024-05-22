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
#include "play_helpers/robot_assignment.hpp"


using namespace ateam_kenobi;  // NOLINT(build/namespaces)
using namespace ateam_kenobi::play_helpers;  // NOLINT(build/namespaces)

using testing::ElementsAre;
using testing::Optional;
using testing::Field;
using testing::Eq;

TEST(RobotAssignmentTest, EmptyRobots)
{
  std::vector<Robot> robots;
  std::vector<ateam_geometry::Point> goals = {
    ateam_geometry::Point(0, 0)
  };
  const auto assignments = assignRobots(robots, goals);
  EXPECT_THAT(assignments, ElementsAre(Eq(std::nullopt)));
}

TEST(RobotAssignmentTest, EmptyGoals)
{
  std::vector<Robot> robots = {
    {}
  };
  std::vector<ateam_geometry::Point> goals;
  const auto assignments = assignRobots(robots, goals);
  EXPECT_TRUE(assignments.empty());
}

TEST(RobotAssignmentTest, OneRobotOneGoal)
{
  std::vector<Robot> robots {
    {1, true, true, ateam_geometry::Point(1, 2), 0.0, ateam_geometry::Vector{}, 0.0, false, true,
      false}
  };
  std::vector<ateam_geometry::Point> goals = {
    ateam_geometry::Point(3, 4)
  };
  const auto assignments = assignRobots(robots, goals);
  EXPECT_THAT(assignments, ElementsAre(Optional(Field(&Robot::id, Eq(1)))));
}

TEST(RobotAssignmentTest, TwoRobotsOneGoal)
{
  std::vector<Robot> robots {
    {1, true, true, ateam_geometry::Point(1, 2), 0.0, ateam_geometry::Vector{}, 0.0, false, true,
      false},
    {2, true, true, ateam_geometry::Point(3, 4), 0.0, ateam_geometry::Vector{}, 0.0, false, true,
      false}
  };
  std::vector<ateam_geometry::Point> goals = {
    ateam_geometry::Point(3, 4)
  };
  const auto assignments = assignRobots(robots, goals);
  EXPECT_THAT(assignments, ElementsAre(Optional(Field(&Robot::id, Eq(2)))));
}

TEST(RobotAssignmentTest, TwoRobotsTwoGoals)
{
  std::vector<Robot> robots {
    {1, true, true, ateam_geometry::Point(1, 2), 0.0, ateam_geometry::Vector{}, 0.0, false, true,
      false},
    {2, true, true, ateam_geometry::Point(3, 4), 0.0, ateam_geometry::Vector{}, 0.0, false, true,
      false}
  };
  std::vector<ateam_geometry::Point> goals = {
    ateam_geometry::Point(0, 0),
    ateam_geometry::Point(3, 4)
  };
  const auto assignments = assignRobots(robots, goals);
  EXPECT_THAT(
    assignments,
    ElementsAre(Optional(Field(&Robot::id, Eq(1))), Optional(Field(&Robot::id, Eq(2)))));
}

TEST(RobotAssignmentTest, TwoRobotsSameDistance)
{
  std::vector<Robot> robots {
    {1, true, true, ateam_geometry::Point(0, 1), 0.0, ateam_geometry::Vector{}, 0.0, false, true,
      false},
    {2, true, true, ateam_geometry::Point(0, -1), 0.0, ateam_geometry::Vector{}, 0.0, false, true,
      false}
  };
  std::vector<ateam_geometry::Point> goals = {
    ateam_geometry::Point(1, 0),
    ateam_geometry::Point(-1, 0)
  };
  const auto assignments = assignRobots(robots, goals);
  EXPECT_THAT(
    assignments,
    ElementsAre(Optional(Field(&Robot::id, Eq(1))), Optional(Field(&Robot::id, Eq(2)))));
}

TEST(RobotAssignmentTest, DisallowAssigningDisallowedRobots)
{
  std::vector<Robot> robots {
    {1, true, true, ateam_geometry::Point(0, 0), 0.0, ateam_geometry::Vector{}, 0.0, false, true,
      false}
  };
  std::vector<ateam_geometry::Point> goals = {
    ateam_geometry::Point(0, 0)
  };
  std::vector<std::vector<int>> disallowed_ids = {
    {1}
  };
  const auto assignments = assignRobots(robots, goals, disallowed_ids);
  EXPECT_THAT(assignments, testing::ElementsAre(Eq(std::nullopt)));
}
