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

// TODO(Christian) : Fix this
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
    return BehaviorGoal goalie {
        BehaviorGoal::Type::MoveToPoint,
        BehaviorGoal::Priority::Required,
        MoveParam(_goalie_point)
    };
}

std::vector<BehaviorGoal> get_defense_behavior_goals(const World & world, const Field & field, const int & num_defenders){
    std::vector<BehaviorGoal> defenders;
    std::optional<Eigen::Vector2d> ball_location = world.get_unique_ball();
    // Get line between the ball and the goal
    while (!ball_location.has_value()){
        ball_location = world.get_unique_ball();
    }
    ateam_geometry::Point ball = ateam_geometry::EigenToPoint(ball_location);
    ateam_geometry::Point middle_of_our_goal = (-4.5, 0);
    ateam_geometry::Segement block_line = ateam_geometry::Segment(ball, middle_of_our_goal);
    // Object to generate candidate points on this line to block
    std::vector<ateam_geometry::Point> candidate_points;
    Random_points_on_segment_2<ateam_geometry::Point,ateam_geometry::PointCreator> linePointCreator;
    linePointCreator line_to_goal(ball, middle_of_our_goal);
    // Get two defenders
    while (defenders.length() < 2) {
        candidate_points.reserve(50);
        std::copy_n( line_to_goal, 50, std::back_inserter(candidate_points));
        // Remove any that will cause us to be out of bounds
        ateam_geometry::Point previous_point = ateam_geometry::Point(-100,-100);
        for (ateam_geometry::Point candidate : candidate_points) {
                    if (is_point_in_bounds(candidate)){
                        if (ateam_geometry::Segment(previous_point, candidate).squared_length() > pow(robot_diameter,2))){
                            BehaviorGoal go_to_point {
                                BehaviorGoal::Type::MoveToPoint,
                                BehaviorGoal::Priority::Required,
                                MoveParam(ateam_geometry::PointToEigen(candidate))
                            }
                            points_around_ball.push_back(go_to_point);
                            if (points_around_ball.length() > 1){
                                break;
                            }
                        }
                    }
        }
    }
    // Have robots go to these points
    return defenders;
}

#endif // PLAYS__DEFENSE_HPP