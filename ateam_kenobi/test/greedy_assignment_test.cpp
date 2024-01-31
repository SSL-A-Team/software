

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "robot_assignment.hpp"


using namespace ateam_kenobi;  // NOLINT(build/namespaces)
using namespace ateam_kenobi::robot_assignment;  // NOLINT(build/namespaces)

TEST(GreedyAssignmentTest, EmptyRobots)
{
  std::vector<Robot> robots;
  std::vector<ateam_geometry::Point> goals = {
    ateam_geometry::Point(0,0)
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
    {1, ateam_geometry::Point(1,2)}
  };
  std::vector<ateam_geometry::Point> goals = {
    ateam_geometry::Point(3,4)
  };
  const auto assignments = greedyAssignment(robots, goals);
  EXPECT_THAT(assignments, testing::UnorderedElementsAre(testing::Pair(1,0)));
}

TEST(GreedyAssignmentTest, TwoRobotsOneGoal)
{
  std::vector<Robot> robots {
    {1, ateam_geometry::Point(1,2)},
    {2, ateam_geometry::Point(3,4)}
  };
  std::vector<ateam_geometry::Point> goals = {
    ateam_geometry::Point(3,4)
  };
  const auto assignments = greedyAssignment(robots, goals);
  EXPECT_THAT(assignments, testing::UnorderedElementsAre(testing::Pair(2,0)));
}

TEST(GreedyAssignmentTest, TwoRobotsTwoGoals)
{
  std::vector<Robot> robots {
    {1, ateam_geometry::Point(1,2)},
    {2, ateam_geometry::Point(3,4)}
  };
  std::vector<ateam_geometry::Point> goals = {
    ateam_geometry::Point(0,0),
    ateam_geometry::Point(3,4)
  };
  const auto assignments = greedyAssignment(robots, goals);
  EXPECT_THAT(assignments, testing::UnorderedElementsAre(testing::Pair(1,0), testing::Pair(2,1)));
}

TEST(GreedyAssignmentTest, TwoRobotsSameDistance)
{
  std::vector<Robot> robots {
    {1, ateam_geometry::Point(0,1)},
    {2, ateam_geometry::Point(0,-1)}
  };
  std::vector<ateam_geometry::Point> goals = {
    ateam_geometry::Point(1,0),
    ateam_geometry::Point(-1,0)
  };
  const auto assignments = greedyAssignment(robots, goals);
  EXPECT_THAT(assignments, testing::UnorderedElementsAre(testing::Pair(1,0), testing::Pair(2,1)));
}
