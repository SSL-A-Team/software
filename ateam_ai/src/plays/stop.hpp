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

typedef Creator_uniform_2<double,Point>  Pt_creator;

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
    // Generate 25 random points on the circle
    Random_points_on_circle_2<Point,Pt_creator> circlePointGenerator;
    circlePointGenerator circlePoints(25);
    std::vector<Point> candidate_points;
    // Remove any that will cause us to be out of bounds
    // Sort the boundaries by x and y, get the highest + lowest values
    double minX;
    double maxX;
    double minY;
    double maxY;
    for (Point candidate : circlePointGenerator) {
        if (candidate.x() >  maxX || candidate.x() < minX){
            continue;  
        }
        if (candidate.y() > maxY || candidate.y() < minY) {
            continue;
        }
        candidate_points.push_back(candidate);
    }
    // Pick the closest x robots
    // Tell other robots to get on our side of the field and block the goal
}

#endif // PLAYS__STOP_HPP