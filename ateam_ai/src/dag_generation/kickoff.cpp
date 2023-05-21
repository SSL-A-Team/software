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

#include "util/directed_graph.hpp"
#include "types/behavior_goal.hpp"
#include "types/world.hpp"
#include "dag_generation/kickoff.hpp"

DirectedGraph<BehaviorGoal> setup_our_kickoff(const World & world) {
    DirectedGraph<BehaviorGoal> our_kickoff;

    // Have the kicker robot go to the edge of the center circle to prepare for kick
    BehaviorGoal kicker_setup {
        BehaviorGoal::Type::MoveToPoint,
        BehaviorGoal::Priority::Required,
        MoveParam(Eigen::Vector2d{robot_position_.x(), robot_position_.y()})
    }

    // Have the goalie defend the goal

    // Generate optional defenders for the rest of the robots that might exist

};

DirectedGraph<BehaviorGoal> setup_their_kickoff(const World & world) {
    DirectedGraph<BehaviorGoal> their_kickoff;
};