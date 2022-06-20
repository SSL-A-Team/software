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

DirectedGraph<BehaviorFeedback> BehaviorRealization::realize_behaviors(
  const DirectedGraph<Behavior> & behaviors, const World & world)
{
  DirectedGraph<BehaviorFeedback> behavior_results;

  // Assign robots to behaviors
  // Calculate the BehaviorFeedback and build the same directed graph shape


  // Just assign robots in number order if they are available
  std::size_t robot_id = 0;
  for (const auto & root_id : behaviors.get_root_nodes()) {
    // Assume there are no children for now
    const auto & root_node = behaviors.get_node(root_id);

    while (robot_id < world.our_robots.size() && !world.our_robots.at(robot_id).has_value()) {
      robot_id++;
    }
    if (robot_id < world.our_robots.size()) {
      BehaviorFeedback feedback =
        trajectory_generation.get_feedback_from_behavior(
        root_node,
        robot_id,
        world);
      behavior_results.add_node(feedback);
      robot_id++;
    } else {
      // If we don't have enough robots, show the behavior isn't assigned
      behavior_results.add_node(BehaviorFeedback());
    }
  }

  return behavior_results;
}
