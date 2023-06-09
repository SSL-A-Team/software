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

#include "behavior/behavior_realization.hpp"

#include <gtest/gtest.h>

BehaviorGoal create_behavior(BehaviorGoal::Priority priority) {
  return BehaviorGoal(BehaviorGoal::Type::MoveToPoint, priority, MoveParam(Eigen::Vector2d{0, 0}));
}

World create_world_with_x_robots(int num_robots) {
  World w;
  for (int i = 0; i < num_robots; i++) {
    w.our_robots.at(i) = Robot();
  }
  return w;
}

BehaviorPlan create_behavior_plan_with_t_end(double t_end) {
  BehaviorPlan bp;
  bp.trajectory.samples.push_back(Sample3d{.time = t_end});

  return bp;
}

TEST(BehaviorRealization, get_priority_to_assignment_group_ReturnEmpty_WhenNoBehaviors)
{
  BehaviorRealization realization;
  DirectedGraph<BehaviorGoal> behaviors;

  BehaviorRealization::PriorityGoalListMap ret = realization.get_priority_to_assignment_group(behaviors);

  ASSERT_EQ(ret.size(), 3);
  EXPECT_TRUE(ret.at(BehaviorGoal::Priority::Required).empty());
  EXPECT_TRUE(ret.at(BehaviorGoal::Priority::Medium).empty());
  EXPECT_TRUE(ret.at(BehaviorGoal::Priority::Low).empty());
}

TEST(BehaviorRealization, get_priority_to_assignment_group_ReturnRequired_WhenOnlyRootRequired)
{
  BehaviorRealization realization;
  DirectedGraph<BehaviorGoal> behaviors;
  BehaviorRealization::BehaviorGoalNodeIdx beh1 = behaviors.add_node(create_behavior(BehaviorGoal::Required));
  BehaviorRealization::BehaviorGoalNodeIdx beh2 = behaviors.add_node(create_behavior(BehaviorGoal::Required));

  BehaviorRealization::PriorityGoalListMap ret = realization.get_priority_to_assignment_group(behaviors);

  ASSERT_EQ(ret.size(), 3);
  ASSERT_EQ(ret.at(BehaviorGoal::Priority::Required).size(), 2);
  EXPECT_EQ(ret.at(BehaviorGoal::Priority::Required).at(0), beh1);
  EXPECT_EQ(ret.at(BehaviorGoal::Priority::Required).at(1), beh2);
  EXPECT_TRUE(ret.at(BehaviorGoal::Priority::Medium).empty());
  EXPECT_TRUE(ret.at(BehaviorGoal::Priority::Low).empty());
}

TEST(BehaviorRealization, get_priority_to_assignment_group_ReturnRequired_WhenOnlyNestedRequired)
{
  BehaviorRealization realization;
  DirectedGraph<BehaviorGoal> behaviors;
  BehaviorRealization::BehaviorGoalNodeIdx beh1 = behaviors.add_node(create_behavior(BehaviorGoal::Required));
  BehaviorRealization::BehaviorGoalNodeIdx beh2 = behaviors.add_node(create_behavior(BehaviorGoal::Required), beh1);

  BehaviorRealization::PriorityGoalListMap ret = realization.get_priority_to_assignment_group(behaviors);

  ASSERT_EQ(ret.size(), 3);
  ASSERT_EQ(ret.at(BehaviorGoal::Priority::Required).size(), 2);
  EXPECT_EQ(ret.at(BehaviorGoal::Priority::Required).at(0), beh1);
  EXPECT_EQ(ret.at(BehaviorGoal::Priority::Required).at(1), beh2);
  EXPECT_TRUE(ret.at(BehaviorGoal::Priority::Medium).empty());
  EXPECT_TRUE(ret.at(BehaviorGoal::Priority::Low).empty());
}

TEST(BehaviorRealization, get_priority_to_assignment_group_ReturnAll_WhenAll)
{
  BehaviorRealization realization;
  DirectedGraph<BehaviorGoal> behaviors;
  BehaviorRealization::BehaviorGoalNodeIdx beh1 = behaviors.add_node(create_behavior(BehaviorGoal::Required));
  BehaviorRealization::BehaviorGoalNodeIdx beh2 = behaviors.add_node(create_behavior(BehaviorGoal::Required), beh1);
  BehaviorRealization::BehaviorGoalNodeIdx beh3 = behaviors.add_node(create_behavior(BehaviorGoal::Medium));
  BehaviorRealization::BehaviorGoalNodeIdx beh4 = behaviors.add_node(create_behavior(BehaviorGoal::Medium), beh3);
  BehaviorRealization::BehaviorGoalNodeIdx beh5 = behaviors.add_node(create_behavior(BehaviorGoal::Low));
  BehaviorRealization::BehaviorGoalNodeIdx beh6 = behaviors.add_node(create_behavior(BehaviorGoal::Low), beh5);

  BehaviorRealization::PriorityGoalListMap ret = realization.get_priority_to_assignment_group(behaviors);

  ASSERT_EQ(ret.size(), 3);
  ASSERT_EQ(ret.at(BehaviorGoal::Priority::Required).size(), 2);
  EXPECT_EQ(ret.at(BehaviorGoal::Priority::Required).at(0), beh1);
  EXPECT_EQ(ret.at(BehaviorGoal::Priority::Required).at(1), beh2);
  ASSERT_EQ(ret.at(BehaviorGoal::Priority::Medium).size(), 2);
  EXPECT_EQ(ret.at(BehaviorGoal::Priority::Medium).at(0), beh3);
  EXPECT_EQ(ret.at(BehaviorGoal::Priority::Medium).at(1), beh4);
  ASSERT_EQ(ret.at(BehaviorGoal::Priority::Low).size(), 2);
  EXPECT_EQ(ret.at(BehaviorGoal::Priority::Low).at(0), beh5);
  EXPECT_EQ(ret.at(BehaviorGoal::Priority::Low).at(1), beh6);
}

TEST(BehaviorRealization, get_available_robots_ReturnX_WhenX)
{
  for (int i = 0; i < 16; i++) {
    BehaviorRealization realization;
    World w = create_world_with_x_robots(i);

    std::set<BehaviorRealization::RobotID> ret = realization.get_available_robots(w);

    ASSERT_EQ(ret.size(), i);
    for (int j = 0; j < 16; j++) {
      EXPECT_EQ(ret.count(j) > 0, j < i ? 1 : 0);
    }
  }
}

TEST(BehaviorRealization, get_available_robots_ReturnsCorrectMapping_WhenNonZero)
{
  BehaviorRealization realization;
  DirectedGraph<BehaviorGoal> behaviors;
  std::vector<BehaviorRealization::BehaviorGoalNodeIdx> idxs_to_plan;
  for (int i = 0; i < 16; i++) {
    behaviors.add_node(create_behavior(BehaviorGoal::Required));
    idxs_to_plan.push_back(i);
  }

  std::set<BehaviorRealization::RobotID> available_robots;
  for (int i = 0; i < 16; i++) {
    available_robots.insert(i);
  }

  std::map<BehaviorRealization::RobotID, int> count_of_calls;
  for (int i = 0; i < 16; i++) {
    count_of_calls[i] = 0;
  }
  auto plan_fnc = [&count_of_calls](BehaviorGoal goal, int robot_id, const World & world) {
    count_of_calls.at(robot_id)++;
    return BehaviorPlan();
  };

  BehaviorRealization::CandidatePlans ret = realization.generate_candidate_plans(
    idxs_to_plan, available_robots, behaviors, create_world_with_x_robots(16), plan_fnc);

  for (auto call_count_pair : count_of_calls) {
    EXPECT_EQ(call_count_pair.second, idxs_to_plan.size());
  }
}

TEST(BehaviorRealization, assign_goals_to_plans_ReturnEmpty_WhenEmpty) {
  BehaviorRealization realization;

  BehaviorRealization::GoalToPlanMap ret = realization.assign_goals_to_plans(
    {}, {}, {{}}, World());

  EXPECT_EQ(ret.size(), 0);
}

TEST(BehaviorRealization, assign_goals_to_plans_ReturnCorrect) {
  BehaviorRealization realization;
  std::vector<BehaviorRealization::BehaviorGoalNodeIdx> goals_to_assign{0, 2};
  std::set<BehaviorRealization::RobotID> available_robots{1, 3};
  BehaviorRealization::CandidatePlans candidate_plans;
  candidate_plans[1][0] = create_behavior_plan_with_t_end(100);
  candidate_plans[1][2] = create_behavior_plan_with_t_end(10);
  candidate_plans[3][0] = create_behavior_plan_with_t_end(11);
  candidate_plans[3][2] = create_behavior_plan_with_t_end(100);
  World world;
  world.current_time = 0.0;

  BehaviorRealization::GoalToPlanMap ret = realization.assign_goals_to_plans(goals_to_assign, available_robots, candidate_plans, world);

  EXPECT_EQ(ret.size(), 2);
  EXPECT_EQ(ret.at(0).trajectory.samples.front().time, 11);
  EXPECT_EQ(ret.at(0).assigned_robot_id.value(), 3);
  EXPECT_EQ(ret.at(2).trajectory.samples.front().time, 10);
  EXPECT_EQ(ret.at(2).assigned_robot_id.value(), 1);
}

TEST(BehaviorRealization, assign_goals_to_plans_ReturnCorrect) {
}
