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

#include "types/behavior_plan.hpp"
#include "trajectory_generation/trajectory_editor.hpp"

BehaviorExecutor::BehaviorExecutor(BehaviorRealization & behavior_realization)
: behavior_realization(behavior_realization) {}

std::array<std::optional<Trajectory>, 16> BehaviorExecutor::execute_behaviors(
  const DirectedGraph<BehaviorGoal> & behaviors,
  const World & world,
  BehaviorExecutorState & self_state)
{
  //
  // Grab trajectories for everything
  //
  DirectedGraph<BehaviorPlan> behavior_feedback = behavior_realization.realize_behaviors(
    behaviors, world);

  //
  // Replan course trajectories as we approach their start time
  //

  // Long term planning (>10 seconds) is not valid due to the speed of robots
  // as we approach some of the later behaviors in time, we need to replan them
  // using better planners to dodge these obstacles and everything

  //
  // Send trajectories to follow
  //

  // Clear world trajectories
  for (auto & trajectory : self_state.previous_trajectories) {
    trajectory.reset();
  }

  std::array<std::optional<Trajectory>, 16> output_trajectories;
  for (const auto & root_id : behavior_feedback.get_root_nodes()) {
    // Assume there are no children for now
    const auto & root_node = behavior_feedback.get_node(root_id);

    // If the behavior is assigned a robot
    if (root_node.assigned_robot_id.has_value()) {
      Trajectory trajectory = root_node.trajectory;
      std::size_t assigned_robot = root_node.assigned_robot_id.value();

      if (self_state.previous_trajectories.at(assigned_robot).has_value()) {
        trajectory = trajectory_editor::apply_immutable_duration(
          self_state.previous_trajectories.at(assigned_robot).value(),
          trajectory,
          world.immutable_duration,
          world.current_time);
      }

      output_trajectories.at(root_node.assigned_robot_id.value()) = root_node.trajectory;
      self_state.previous_trajectories.at(assigned_robot) = trajectory;
    }
  }

  return output_trajectories;
}
