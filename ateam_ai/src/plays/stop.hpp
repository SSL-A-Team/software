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


#ifndef PLAYS__STOP_HPP_
#define PLAYS__STOP_HPP_

#include <Eigen/Dense>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/point_generators_2.h>

#include "util/directed_graph.hpp"
#include "types/behavior_goal.hpp"
#include "types/field.hpp"
#include "types/world.hpp"

#include "ateam_geometry/ateam_geometry.hpp"

#include "defense.hpp"
#include "play_helpers.hpp"

typedef Creator_uniform_2<double,Point>  Pt_creator;

DirectedGraph<BehaviorGoal> generate_stop(const World & world, const Field & field, const FieldSidedInfo & our_side_info){
    DirectedGraph<BehaviorGoal> stop;

    auto goalie = get_goalie_behavior_goal(our_side_info);
    stop.add_node(goalie);

    auto attacker_behaviors = generate_points_around_ball(world, field, 3);
    for (BehaviorGoal behavior : attacker_behaviors) {
        stop.add_node(behavior);
    }

    auto defense_behaviors = get_defense_behavior_goals(world, field, 2);
    for (BehaviorGoal behavior : defense_behaviors) {
        stop.add_node(behavior);
    }
    return stop;
}

std::vector<BehaviorGoal> generate_points_around_ball(const World & world, const Field & field, const int num_points){
    std::vector<BehaviorGoal> points_around_ball;
    // Where is the ball?
    std::optional<Eigen::Vector2d> ball_location = world.get_unique_ball();
    while (!ball_location.has_value()){
        ball_location = world.get_unique_ball();
    }
    // Tell 3 robots (attackers) to get close to the ball
    // We must be at least 0.5 m from the ball
    ateam_geometry::Point ball = ateam_geometry::EigenToPoint(ball_location);
    ateam_geometry::Circle circular_boundary = ateam_geometry::makeCircle(ball, 0.5);
    // Generate n + 20 random points on the circle
    Random_points_on_circle_2<Point,Pt_creator> circlePointGenerator;
    circlePointGenerator circlePoints(num_points + 20);
    std::vector<ateam_geometry::Point> candidate_points;
    // Remove any that will cause us to be out of bounds
    for (ateam_geometry::Point candidate : circlePointGenerator) {
        if (is_point_in_bounds(candidate)){
            candidate_points.push_back(candidate);
        }
    }
    // Just get enough points around the ball for the required number of robots
    for (int i = 0; i < num_points; ++i) {
        BehaviorGoal go_to_point {
            BehaviorGoal::Type::MoveToPoint,
            BehaviorGoal::Priority::Required,
            MoveParam(ateam_geometry::PointToEigen(candidate_points[i]));
        }
        points_around_ball.push_back(go_to_point);
    }
    return points_around_ball;
}

#endif // PLAYS__STOP_HPP