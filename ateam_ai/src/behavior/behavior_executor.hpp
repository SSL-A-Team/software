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

#ifndef BEHAVIOR__BEHAVIOR_EXECUTOR_HPP_
#define BEHAVIOR__BEHAVIOR_EXECUTOR_HPP_

#include "behavior/behavior.hpp"
#include "behavior/behavior_feedback.hpp"
#include "behavior/behavior_realization.hpp"

class BehaviorExecutor
{
public:
  void execute_behaviors(
    const DirectedGraph<Behavior> & behaviors,
    BehaviorRealization & behavior_realization)
  {
    //
    // Grab trajectories for everything
    //
    DirectedGraph<BehaviorFeedback> behavior_feedback = behavior_realization.realize_behaviors(
      behaviors);

    // Ideally, convert the behavior graph into a per robot list of trajectories as a function of
    // time
    //    https://help.perforce.com/visualization/jviews/documentation/userman/gantt/images/gen_gantt_default.png
    // each row is a robot
    // columns are a function of time
    // a trajectory fills section of columns for a single robot based on it's start/end time

    //
    // Replan course trajectories as we approach their start time
    //

    // Long term planning (>10 seconds) is not valid due to the speed of robots
    // as we approach some of the later behaviors in time, we need to replan them
    // using better planners to dodge these obsticles and everything

    //
    // Follow trajectories
    //

    // send commands down to motion control
  }
};

#endif  // BEHAVIOR__BEHAVIOR_EXECUTOR_HPP_
