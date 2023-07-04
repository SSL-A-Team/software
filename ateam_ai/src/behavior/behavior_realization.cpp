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

#include <map>
#include <vector>
#include <utility>
#include <deque>
#include <functional>

#include "behavior/behavior_realization.hpp"

#include <ateam_common/assignment.hpp>
#include <ateam_common/status.hpp>

DirectedGraph<BehaviorPlan> BehaviorRealization::realize_behaviors(
  const DirectedGraph<BehaviorGoal> & behaviors, const World & world)
{
  return realize_behaviors_impl(behaviors, world,  std::bind_front(&trajectory_generation::TrajectoryGenerator::GetPlanFromGoal, trajectory_generator));
}

DirectedGraph<BehaviorPlan> BehaviorRealization::realize_behaviors_impl(
  const DirectedGraph<BehaviorGoal> & behaviors, const World & world,
  const GetPlanFromGoalFnc & GetPlanFromGoal)
{
  // Generate list of available robots
  std::set<RobotID> available_robots = get_available_robots(world);

  // Collect nodes by priority
  PriorityGoalListMap priority_to_assignment_group = get_priority_to_assignment_group(behaviors);

  // Remove reserved robots from available (reservation comes from nodes)
  // Could do this when we hit section below but if someone changes
  // order of priorities that breaks down
  for (const auto & behavior_idx : priority_to_assignment_group[BehaviorGoal::Priority::Reserved]) {
    available_robots.erase(behaviors.get_node(behavior_idx).reserved_robot_id);
  }
  // TODO(Cavidano) Define an .at on the behavior dag

  std::map<BehaviorGoalNodeIdx, BehaviorPlan> assigned_goals_to_plans;

  auto generate_plans_and_assign =
    [this, &behaviors, &world, &GetPlanFromGoal, &available_robots, &priority_to_assignment_group,
      &assigned_goals_to_plans](std::vector<BehaviorGoalNodeIdx> goals_to_assign) {
      if (available_robots.empty()) {
        return;
      }

      CandidatePlans candidate_plans = generate_candidate_plans(
        goals_to_assign, available_robots, behaviors, world, GetPlanFromGoal);
      GoalToPlanMap goal_to_plans = assign_goals_to_plans(
        goals_to_assign, available_robots, candidate_plans, world);

      for (const auto & [goal_idx, plan] : goal_to_plans) {
        assigned_goals_to_plans[goal_idx] = plan;
        available_robots.erase(plan.assigned_robot_id.value());
      }
    };

  // Forced singular plan per robot could just use above, But barulic and I both thought we didnt
  // need to call through all of that
  auto generate_reserved_plans =
    [this, &behaviors, &world, &GetPlanFromGoal, &assigned_goals_to_plans]
      (std::vector<BehaviorGoalNodeIdx> goals_to_assign) {
      for (const auto & goal_idx : goals_to_assign) {
        assigned_goals_to_plans[goal_idx] = GetPlanFromGoal(
          behaviors.get_node(goal_idx),
          behaviors.get_node(goal_idx).reserved_robot_id,
          world);
      }
    };

  // TODO(Cavidano) switch statement later
  // Assign all robots to goals accoring to the goal priority
  for (const auto & [priority, list_of_goal_idxs_at_priority] : priority_to_assignment_group) {
    // For each required node, assign independently
    if (priority == BehaviorGoal::Priority::Required) {
      for (const auto & goal_idx : list_of_goal_idxs_at_priority) {
        generate_plans_and_assign({goal_idx});
      }
    } else if (priority == BehaviorGoal::Priority::Reserved) {
      generate_reserved_plans(list_of_goal_idxs_at_priority);
    } else {
      generate_plans_and_assign(list_of_goal_idxs_at_priority);
    }
  }

  return behaviors.copy_shape_with_new_type(assigned_goals_to_plans);
}

BehaviorRealization::PriorityGoalListMap BehaviorRealization::get_priority_to_assignment_group(
  const DirectedGraph<BehaviorGoal> & behaviors)
{
  PriorityGoalListMap priority_to_assignment_group{
    {BehaviorGoal::Priority::Reserved, {}},
    {BehaviorGoal::Priority::Required, {}},
    {BehaviorGoal::Priority::Medium, {}},
    {BehaviorGoal::Priority::Low, {}}
  };

  std::vector<BehaviorGoalNodeIdx> root_nodes = behaviors.get_root_nodes();
  for (auto root_node_id : behaviors.get_root_nodes()) {
    std::deque<BehaviorGoalNodeIdx> nodes_to_add{root_node_id};
    while (nodes_to_add.size() > 0) {
      BehaviorGoalNodeIdx front_node = nodes_to_add.front();
      nodes_to_add.pop_front();

      BehaviorGoal behavior = behaviors.get_node(front_node);
      priority_to_assignment_group.at(behavior.priority).emplace_back(front_node);

      std::vector<BehaviorGoalNodeIdx> children = behaviors.get_children(front_node);
      for (const auto & child_node_id : children) {
        nodes_to_add.emplace_back(child_node_id);
      }
    }
  }

  return priority_to_assignment_group;
}

std::set<BehaviorRealization::RobotID> BehaviorRealization::get_available_robots(
  const World & world)
{
  std::set<RobotID> available_robots;
  for (std::size_t i = 0; i < 15; i++) {
    if (world.our_robots.at(i).has_value()) {
      available_robots.insert(i);
    }
  }

  return available_robots;
}

BehaviorRealization::CandidatePlans BehaviorRealization::generate_candidate_plans(
  const std::vector<BehaviorGoalNodeIdx> & goals_node_idxs_to_assign,
  const std::set<RobotID> & available_robots,
  const DirectedGraph<BehaviorGoal> & behaviors,
  const World & world,
  const GetPlanFromGoalFnc & GetPlanFromGoal)
{
  CandidatePlans candidate_plans;
  for (const auto & robot_id : available_robots) {
    for (const auto & goal_node_idx : goals_node_idxs_to_assign) {
      candidate_plans[robot_id][goal_node_idx] =
        GetPlanFromGoal(
        behaviors.get_node(goal_node_idx),
        robot_id,
        world);
    }
  }

  return candidate_plans;
}

BehaviorRealization::GoalToPlanMap BehaviorRealization::assign_goals_to_plans(
  const std::vector<BehaviorGoalNodeIdx> & goals_to_assign,
  const std::set<BehaviorRealization::RobotID> & available_robots,
  const CandidatePlans & candidate_plans,
  const World & world)
{
  if (goals_to_assign.empty() || available_robots.empty()) {
    return {};
  }

  ATEAM_CHECK(candidate_plans.size() == available_robots.size(), "Must have one plan per robot");
  ATEAM_CHECK(
    candidate_plans.begin()->second.size() == goals_to_assign.size(),
    "Must have one plan per goal");

  // Create cost matrix
  std::map<std::size_t, RobotID> row_idx_to_robot_id;
  std::map<std::size_t, BehaviorGoalNodeIdx> col_idx_to_goal_node_idx;
  Eigen::MatrixXd robot_to_goal_cost_matrix(available_robots.size(), goals_to_assign.size());

  int robot_id_idx = 0;
  for (const auto & robot_id : available_robots) {
    int goal_idx = 0;
    for (const auto & goal : goals_to_assign) {
      robot_to_goal_cost_matrix(robot_id_idx, goal_idx) = cost(
        candidate_plans.at(robot_id).at(
          goal), world);
      col_idx_to_goal_node_idx[goal_idx] = goal;
      goal_idx++;
    }
    row_idx_to_robot_id[robot_id_idx] = robot_id;
    robot_id_idx++;
  }

  // Do assignment
  auto assignment = ateam_common::assignment::optimize_assignment(robot_to_goal_cost_matrix);

  // Map cost matrix row/col back to behavior plans / robots
  GoalToPlanMap assigned_goals_to_plans;
  for (const auto & [row_idx, col_idx] : assignment) {
    RobotID robot_to_assign = row_idx_to_robot_id.at(row_idx);
    BehaviorGoalNodeIdx goal_to_assign = col_idx_to_goal_node_idx.at(col_idx);

    assigned_goals_to_plans[goal_to_assign] =
      candidate_plans.at(robot_to_assign).at(goal_to_assign);
    assigned_goals_to_plans[goal_to_assign].assigned_robot_id = robot_to_assign;
  }

  return assigned_goals_to_plans;
}

double BehaviorRealization::cost(const BehaviorPlan & bp, const World & world)
{
  if(bp.trajectory.samples.empty()) {
    // return std::numeric_limits<double>::infinity();
    return 1000.0;
  }
  return bp.trajectory.samples.back().time - world.current_time;
}
