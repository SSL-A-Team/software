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

#include "behavior/behavior.hpp"
#include "behavior/behavior_feedback.hpp"
#include "util/directed_graph.hpp"

class BehaviorRealization
{
public:
  /**
   * Given a set of behaviors, return trajectories and timings of all the behaviors expected for execution
   *
   * @note Simple version that's easy to implement initially
   */
  DirectedGraph<BehaviorFeedback> realize_behaviors(const DirectedGraph<Behavior> & behaviors);

private:
  void traverse_and_assign_behaviors(
    const DirectedGraph<Behavior> & behaviors,
    DirectedGraph<BehaviorFeedback> & behavior_results,
    std::size_t behavior_parent,
    std::size_t results_parent);
};

#endif  // BEHAVIOR__BEHAVIOR_REALIZATION_HPP_
