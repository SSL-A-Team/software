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

#include <vector>
#include <cmath>

#include "util/directed_graph.hpp"
#include "types/behavior_goal.hpp"
#include "types/world.hpp"

#include "ateam_geometry/ateam_geometry.hpp"
#include "ateam_common/robot_constants.hpp"

#include "defense.hpp"
#include "play_helpers.hpp"

std::vector<BehaviorGoal> generate_points_around_ball(
  const World & world,
  const int num_points)
{
  std::vector<BehaviorGoal> points_around_ball;
  // Where is the ball?
  std::optional<Ball> ball = world.get_unique_ball();
  if (!ball.has_value()) {
    return points_around_ball;
  }
  ateam_geometry::Point ball_location = ateam_geometry::EigenToPoint(ball.value().pos);
  // Used to create points around a circle with radius 0.5 m
  typedef CGAL::Random_points_on_circle_2<ateam_geometry::Point,
    ateam_geometry::PointCreator> circlePointGenerator;
  circlePointGenerator circlePoints(0.55);
  // We need 3 robots (attackers) to get close to the ball
  while (static_cast<int>(points_around_ball.size()) < 3) {
    std::vector<ateam_geometry::Point> candidate_points;
    candidate_points.reserve(50);
    std::copy_n(circlePoints, 50, std::back_inserter(candidate_points));
    // Pick the first points that are within bounds on the circle
    // and a robot's diameter away from each other
    ateam_geometry::Point previous_point = ateam_geometry::Point(-100, -100);
    for (ateam_geometry::Point candidate : candidate_points) {
      if (is_point_in_bounds(candidate, world.field)) {
        if (ateam_geometry::Segment(
            previous_point,
            candidate).squared_length() > pow(kRobotDiameter, 2))
        {
          BehaviorGoal go_to_point {
            BehaviorGoal::Type::MoveToPoint,
            BehaviorGoal::Priority::Required,
            MoveParam(ateam_geometry::PointToEigen(candidate))
          };
          points_around_ball.push_back(go_to_point);
          if (static_cast<int>(points_around_ball.size()) > 2) {
            break;
          }
          previous_point = candidate;
        }
      }
    }
  }
  return points_around_ball;
}

DirectedGraph<BehaviorGoal> generate_stop(
  const World & world)
{
  DirectedGraph<BehaviorGoal> stop;

  auto goalie = get_goalie_behavior_goal(world);
  stop.add_node(goalie);

  auto attacker_behaviors = generate_points_around_ball(world, 3);
  for (BehaviorGoal behavior : attacker_behaviors) {
    stop.add_node(behavior);
  }

  auto defense_behaviors = get_defense_behavior_goals(world, 2);
  for (BehaviorGoal behavior : defense_behaviors) {
    stop.add_node(behavior);
  }
  return stop;
}

#endif  // PLAYS__STOP_HPP_
