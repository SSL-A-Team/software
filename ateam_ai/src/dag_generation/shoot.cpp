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

#include "shoot.hpp"
#include "util/directed_graph.hpp"
#include "types/behavior_goal.hpp"
#include <Eigen/Dense>

DirectedGraph<BehaviorGoal> generate_basic_shoot(
    const World & world, const FieldSidedInfo & their_field_side
    ){
    DirectedGraph<BehaviorGoal> basic_shoot;

    // Get the center of the opponent's goal from the field geometry
    Eigen::Vector2d _goal_center = ( 
        (their_field_side.goal_posts.at(0).x()  + their_field_side.goal_posts.at(1).x()) / 2,
        their_field_side.goal_posts.at(0).y() // Will change this to be inside of goal
    )
    // Tell one robot to shoot
    BehaviorGoal shoot {
        BehaviorGoal::Type::PivotKick,
        BehaviorGoal::Priority::Required,
        // Kick to the center of their goal
        KickParam(_goal_center)
    };
    basic_shoot.add_node(shoot);

    return basic_shoot;
};