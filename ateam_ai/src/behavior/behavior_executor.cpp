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

#include "behavior/behavior_executor.hpp"

#include "behavior/behavior_feedback.hpp"
#include "trajectory_generation/trajectory_editor.hpp"

BehaviorExecutor::BehaviorExecutor(BehaviorRealization & behavior_realization)
: behavior_realization(behavior_realization) {}

std::array<std::optional<Trajectory>, 16> BehaviorExecutor::execute_behaviors(
  const DirectedGraph<Behavior> & behaviors,
  const World & world,
  BehaviorExecutorState & self_state)
{
  //
  // Grab trajectories for everything
  //
  DirectedGraph<BehaviorFeedback> behavior_feedback = behavior_realization.realize_behaviors(
    behaviors, world);

  //
  // Replan course trajectories as we approach their start time
  //

  // Long term planning (>10 seconds) is not valid due to the speed of robots
  // as we approach some of the later behaviors in time, we need to replan them
  // using better planners to dodge these obsticles and everything

  //
  // Send trajectories to follow
  //


  std::array<std::optional<Trajectory>, 16> output_trajectories;
  for (const auto & root_id : behavior_feedback.get_root_nodes()) {
    // Assume there are no children for now
    const auto & root_node = behavior_feedback.get_node(root_id);

    // If the behavior is assigned a robot
    if (root_node.assigned_robot_id.has_value()) {
      Trajectory trajectory = root_node.trajectory;
      std::size_t assigned_robot = root_node.assigned_robot_id.value();

      Sample3d command;
      if (root_node.trajectory.samples.size() >= 2) {
        command = root_node.trajectory.samples.at(1);  // Where I want to be next frame
      } else if (root_node.trajectory.samples.size() == 1) {
        // Already at target since only sample is current position
        command = root_node.trajectory.samples.front();
      } else {
        continue;
      }

      output_trajectories.at(assigned_robot) = root_node.trajectory;
    }
  }

  return output_trajectories;
}
