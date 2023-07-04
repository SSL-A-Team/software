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

#ifndef TRAJECTORY_GENERATION__TRAJECTORY_GENERATION_HPP_
#define TRAJECTORY_GENERATION__TRAJECTORY_GENERATION_HPP_

#include "types/behavior_goal.hpp"
#include "types/behavior_plan.hpp"
#include "types/world.hpp"
#include "rrt_path_planner.hpp"

/**
 * Given a behavior and assigned robot, build a motion plan needed to fully execute it
 */
namespace trajectory_generation
{
class TrajectoryGenerator {
  public:
  BehaviorPlan GetPlanFromGoal(BehaviorGoal behavior, int assigned_robot, const World & world);

  private:
    ateam_ai::trajectory_generation::RrtPathPlanner rrt_path_planner;
};

}  // namespace trajectory_generation

#endif  // TRAJECTORY_GENERATION__TRAJECTORY_GENERATION_HPP_
