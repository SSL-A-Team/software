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

#include "trajectory_generation/trajectory_generation.hpp"

DirectedGraph<BehaviorPlan> BehaviorRealization::realize_behaviors(
  const DirectedGraph<BehaviorGoal> & behaviors, const World & world)
{
  DirectedGraph<BehaviorPlan> behavior_results;

  // Assign robots to behaviors
  auto robot_id_to_behavior_goal = assign_to_behaviors(behaviors, world);

  // Built trajectories for each assigned behavior_goal
  for (const auto & [robot_id, behavior_goal] : robot_id_to_behavior_goal) {
    BehaviorPlan plan =
      trajectory_generation::GetPlanFromGoal(
      behavior_goal,
      robot_id,
      world);
    behavior_results.add_node(plan);
  }

  return behavior_results;
}

std::map<std::size_t, BehaviorGoal> BehaviorRealization::assign_to_behaviors(
  const DirectedGraph<BehaviorGoal> & behaviors, const World & world)
{
  std::map<std::size_t, BehaviorGoal> output;

  // Just assign robots in number order if they are available
  std::size_t robot_id = 0;
  for (const auto & root_id : behaviors.get_root_nodes()) {
    // Assume there are no children for now
    auto root_node = behaviors.get_node(root_id);

    // Get next avaliable robot id
    robot_id = next_avaliable_robot_id(robot_id, world);

    // Is valid id
    if (robot_id < world.our_robots.size()) {
      output.insert(std::make_pair(robot_id, root_node));
      robot_id++;
    }
  }

  return output;
}

std::size_t BehaviorRealization::next_avaliable_robot_id(
  const std::size_t last_assigned_id,
  const World & world)
{
  // Get next avaliable robot id
  std::size_t robot_id = last_assigned_id;
  while (robot_id < world.our_robots.size() && !world.our_robots.at(robot_id).has_value()) {
    robot_id++;
  }

  return robot_id;
}
