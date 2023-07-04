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

#ifndef BEHAVIOR__BEHAVIOR_REALIZATION_HPP_
#define BEHAVIOR__BEHAVIOR_REALIZATION_HPP_

#include <map>
#include <set>
#include <vector>

#include "types/behavior_goal.hpp"
#include "types/behavior_plan.hpp"
#include "types/world.hpp"
#include "util/directed_graph.hpp"
#include "trajectory_generation/trajectory_generation.hpp"

/**
 * Given a set of behaviors
 *  - Assign the behaviors to robots
 *  - Generate trajectories for the robots
 *  - Figure out relative timings to start the trajectories
 */
class BehaviorRealization
{
public:
  /**
   * Generate plans and assign robots to the set of goals in an "optimal" way.
   * Each required behavior is assigned in order of when they are added to the DAG
   * in a depth first way.
   *
   * Non-required nodes at each stage are assigned in a "globally optimial" way for all
   * available robots and goals in each non-required priority level.
   *
   * Note that since the required goals are assigned in a highly greedy way, placing too many
   * required goals will results in "starvation" of any non-required nodes.
  */
  DirectedGraph<BehaviorPlan> realize_behaviors(
    const DirectedGraph<BehaviorGoal> & behaviors,
    const World & world);

  // ---------------- internal ----------------
  using GetPlanFromGoalFnc = std::function<BehaviorPlan(BehaviorGoal, int, const World &)>;
  DirectedGraph<BehaviorPlan> realize_behaviors_impl(
    const DirectedGraph<BehaviorGoal> & behaviors,
    const World & world,
    const GetPlanFromGoalFnc & GetPlanFromGoal);

  using BehaviorGoalNodeIdx = std::size_t;
  using RobotID = std::size_t;
  using Priority = BehaviorGoal::Priority;
  using PriorityGoalListMap = std::map<Priority, std::vector<BehaviorGoalNodeIdx>>;
  using GoalToPlanMap = std::map<BehaviorGoalNodeIdx, BehaviorPlan>;
  using CandidatePlans = std::map<RobotID, std::map<BehaviorGoalNodeIdx, BehaviorPlan>>;

  /**
   * Flattens DAG into a list of goals in each priority
  */
  PriorityGoalListMap get_priority_to_assignment_group(
    const DirectedGraph<BehaviorGoal> & behaviors);

  /**
   * Gets list of robot ids on our team that are available to assign
   * Eg: All robots we have tracking data for
  */
  std::set<RobotID> get_available_robots(const World & world);

  /**
   * Generate a plan for each robot to each goal
  */
  CandidatePlans generate_candidate_plans(
    const std::vector<BehaviorGoalNodeIdx> & goals_nodes_idxs_to_assign,
    const std::set<RobotID> & available_robots,
    const DirectedGraph<BehaviorGoal> & behaviors,
    const World & world,
    const GetPlanFromGoalFnc & GetPlanFromGoal);

  /**
   * Assigns goals to their optimal plans in a globally optimal way
  */
  GoalToPlanMap assign_goals_to_plans(
    const std::vector<BehaviorGoalNodeIdx> & goals_to_assign,
    const std::set<RobotID> & available_robots,
    const CandidatePlans & candidate_plans,
    const World & world);

  double cost(const BehaviorPlan & bp, const World & world);

  private:
  trajectory_generation::TrajectoryGenerator trajectory_generator;
};

#endif  // BEHAVIOR__BEHAVIOR_REALIZATION_HPP_
