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


#ifndef PLAYS__DEFENSE_HPP_
#define PLAYS__DEFENSE_HPP_

#include <Eigen/Dense>
#include <array>

#include "types/world.hpp"
#include "types/behavior_goal.hpp"
#include "ateam_geometry/utilities.hpp"

const FieldSidedInfo & our_side_info
DirectedGraph<BehaviorGoal> generate_basic_defense(const World & world, const FieldSidedInfo & our_side_info){
    DirectedGraph<BehaviorGoal> defense_graph;
    // Go to the middle of the goalie area
    BehaviorGoal goalie = get_goalie_behavior_goal(our_side_info);
    defense_graph.add_node(goalie);
};

BehaviorGoal get_goalie_behavior_goal(const FieldSidedInfo & our_side_info){
    // Sort the corners with top right at index 0 and bottom left at index 2
    std::array<Eigen::Vector2d> defense_area = std::sort(our_side_info.goalie_corners.begin(),
        our_side_info.goalie_corners.end(), ateam_geometry::sort_eigen_2d_high_low());
    
    Eigen::Vector2d _goalie_point = Eigen::Vector2d(
        // Here I'm assuming these are opposite corners of the goal
        // Does this need to be negative to match our conventions?
        (our_side_info.goalie_corners.at(0).x() + our_side_info.goalie_corners.at(2).x()) / 2,
        (our_side_info.goalie_corners.at(0).y() + our_side_info.goalie_corners.at(2).y()) / 2
    );

    // Have the goalie defend the goal
    BehaviorGoal goalie {
        BehaviorGoal::Type::MoveToPoint,
        BehaviorGoal::Priority::Required,
        MoveParam(_goalie_point)
    };
}
#endif // PLAYS__DEFENSE_HPP