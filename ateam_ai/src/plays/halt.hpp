// Copyright 2023 A Team
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

#ifndef PLAYS__HALT_HPP_
#define PLAYS__HALT_HPP_

#include "types/world.hpp"
#include "types/behavior_goal.hpp"

#include <Eigen/Dense>

DirectedGraph<BehaviorGoal> generate_halt(const World & world) {
    DirectedGraph<BehaviorGoal> halt_graph;
    for (std::size_t id = 0; id < world.our_robots.size(); id++) {
        // Generate a required halt for every robot on our team
        BehaviorGoal halt {
            BehaviorGoal::Type::Halt,
            BehaviorGoal::Priority::Required,
            HaltParam()
        };
        halt_graph.add_node(halt);
    }
    return halt_graph;
}
#endif // PLAYS__HALT_HPP