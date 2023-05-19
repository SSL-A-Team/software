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

#include <map>
#include <vector>
#include <utility>
#include <queue>
#include <set>

#include "trajectory_generation/trajectory_generation.hpp"

#include <ateam_common/assignment.hpp>

DirectedGraph<BehaviorPlan> BehaviorRealization::realize_behaviors(
  const DirectedGraph<BehaviorGoal> & behaviors, const World & world)
{
  // See the following for the implementation
  // https://docs.google.com/document/d/1VRBgCGCAwEGkH0RJALpWob7kBlKYvFso6VbcEfTUHZo/edit?usp=sharing

  // Generate list of availabe robots
  std::set<RobotID> available_robots;
  for (std::size_t i = 0; i < 15; i++) {
    if (world.our_robots.at(i).has_value()) {
      available_robots.insert(i);
    }
  }

  // Collect nodes by priority
  std::map<Priority, std::vector<BehaviorGoalNodeIdx>> priority_to_assignment_group{
    {BehaviorGoal::Priority::Required, {}},
    {BehaviorGoal::Priority::Medium, {}},
    {BehaviorGoal::Priority::Low, {}}
  };
  std::vector<BehaviorGoalNodeIdx> root_nodes = behaviors.get_root_nodes();
  for (auto root_node_id : behaviors.get_root_nodes()) {
    std::deque<BehaviorGoalNodeIdx> nodes_to_add{root_node_id};
    while (nodes_to_add.size() > 0) {
      BehaviorGoal behavior = behaviors.get_node(nodes_to_add.front());
      priority_to_assignment_group[behavior.priority].emplace_back(nodes_to_add.front());
      std::vector<BehaviorGoalNodeIdx> children = behaviors.get_children(nodes_to_add.front());
      nodes_to_add.pop_front();

      for (const auto & child_node_id : children) {
        nodes_to_add.emplace_back(child_node_id);
      }
    }
  }

  std::map<BehaviorGoalNodeIdx, BehaviorPlan> assigned_goals_to_plans;

  // Assign each required node independently
  for (const auto & required_node_id : priority_to_assignment_group[BehaviorGoal::Priority::Required]) {
    std::map<RobotID, BehaviorPlan> candidate_plans;
    BehaviorGoal behavior_goal = behaviors.get_node(required_node_id);

    // Quit early if no robots to assign
    if (available_robots.size() == 0) {
      break;
    }

    for (const auto & robot_id : available_robots) {
      candidate_plans[robot_id] =
        trajectory_generation::GetPlanFromGoal(
        behavior_goal,
        robot_id,
        world);
    }

    // Make cost matrix
    // Only 1 role
    Eigen::MatrixXd robot_to_role_cost_matrix(available_robots.size(), 1);
    auto cost = [&world](BehaviorPlan bp) { return bp.trajectory.samples.back().time - world.current_time; };
    std::map<int, RobotID> idx_to_robot_id;
    int idx = 0;
    for (const auto & [robot_id, behavior_plan] : candidate_plans) {
      robot_to_role_cost_matrix(idx, 0) = cost(behavior_plan);
      idx_to_robot_id[idx] = robot_id;
      idx++;
    }

    // Do assignment
    auto assignment = ateam_common::assignment::optimize_assignment(robot_to_role_cost_matrix);

    // There is only 1 role, so we can just grab the first robot id assigned
    RobotID assigned_robot_id = idx_to_robot_id.at(assignment.begin()->first);
    assigned_goals_to_plans[required_node_id] = candidate_plans.at(assigned_robot_id);
    available_robots.erase(assigned_robot_id);
  }

  // Assign each non-required node as a group
  for (const auto & [priority, list_of_non_required_roles] : priority_to_assignment_group) {
    // Skip the required role as we already assigned it
    if (priority == BehaviorGoal::Priority::Required) {
      continue;
    }

    // Quit early if no robots to assign
    if (available_robots.size() == 0) {
      break;
    }
    // Continue to next priority if no roles at this priority
    if (list_of_non_required_roles.size() == 0) {
      continue;
    }

    std::map<RobotID, std::map<BehaviorGoalNodeIdx, BehaviorPlan>> candidate_plans;
    for (const auto & robot_id : available_robots) {
      for (const auto & goal_node_idx : list_of_non_required_roles) {
        BehaviorGoal behavior_goal = behaviors.get_node(goal_node_idx);
        candidate_plans[robot_id][goal_node_idx] =
          trajectory_generation::GetPlanFromGoal(
          behavior_goal,
          robot_id,
          world);
      }
    }

    auto cost = [&world](BehaviorPlan bp) { return bp.trajectory.samples.back().time - world.current_time; };
    Eigen::MatrixXd robot_to_role_cost_matrix(available_robots.size(), list_of_non_required_roles.size());
    std::map<int, RobotID> idx_to_robot_id;
    std::map<int, BehaviorGoalNodeIdx> idx_to_behavior_goal;
    int robot_id_idx = 0;
    for (const auto & robot_id : available_robots) {
      int role_idx = 0;
      for (const auto & role : list_of_non_required_roles) {
        robot_to_role_cost_matrix(robot_id_idx, role_idx) = cost(candidate_plans.at(robot_id).at(role));
        idx_to_behavior_goal[role_idx] = role;
        role_idx++;
      }
      idx_to_robot_id[robot_id_idx] = robot_id;
      robot_id_idx++;
    }

    // Do assignment
    auto assignment = ateam_common::assignment::optimize_assignment(robot_to_role_cost_matrix);

    // Pull out assignemnts and add to assigned_goals_to_plans list
    for (const auto & [robot_id_idx, role_idx] : idx_to_robot_id) {
      RobotID robot_to_assign = idx_to_robot_id.at(robot_id_idx);
      BehaviorGoalNodeIdx goal_to_assign = idx_to_behavior_goal.at(role_idx);

      assigned_goals_to_plans[goal_to_assign] = candidate_plans.at(robot_to_assign).at(goal_to_assign);
      available_robots.erase(robot_to_assign);
    }
  }

  return behaviors.copy_shape_with_new_type(assigned_goals_to_plans);
}
