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

#include <vector>
#include <queue>

void BehaviorRealization::update_world(World world)
{
  this->world = world;
  trajectory_generation.update_world(world);
}

DirectedGraph<BehaviorFeedback> BehaviorRealization::realize_behaviors(
  const DirectedGraph<Behavior> & behaviors)
{
  std::vector<std::size_t> root_nodes = behaviors.get_root_nodes();
  DirectedGraph<BehaviorFeedback> behavior_results;
  std::queue<int> robots_to_assign;

  for (int i = 0; i < 16; i++) {
    robots_to_assign.push(i);
  }

  for (auto root_node : root_nodes) {
    int assigned_robot = robots_to_assign.front();
    robots_to_assign.pop();

    BehaviorFeedback root_feedback =
      trajectory_generation.get_feedback_from_behavior(
      behaviors.get_node(root_node),
      assigned_robot);

    std::size_t root_feedback_idx = behavior_results.add_node(root_feedback);
    traverse_and_assign_behaviors(
      behaviors, behavior_results, root_node, root_feedback_idx,
      robots_to_assign);
  }

  return behavior_results;
}

void BehaviorRealization::traverse_and_assign_behaviors(
  const DirectedGraph<Behavior> & behaviors,
  DirectedGraph<BehaviorFeedback> & behavior_results,
  std::size_t behavior_parent,
  std::size_t results_parent,
  std::queue<int> & robots_to_assign)
{
  std::vector<std::size_t> children = behaviors.get_children(behavior_parent);

  if (children.empty()) {
    return;
  }

  for (const auto & child : children) {
    int assigned_robot = robots_to_assign.front();
    robots_to_assign.pop();

    BehaviorFeedback child_feedback =
      trajectory_generation.get_feedback_from_behavior(
      behaviors.get_node(child),
      assigned_robot);

    std::size_t child_feedback_idx = behavior_results.add_node(child_feedback, results_parent);
    traverse_and_assign_behaviors(
      behaviors, behavior_results, child, child_feedback_idx, robots_to_assign);
  }
}
