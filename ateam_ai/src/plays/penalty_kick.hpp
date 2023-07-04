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

#ifndef PLAYS__PENALTY_KICK_HPP_
#define PLAYS__PENALTY_KICK_HPP_

#include <vector>

#include "types/behavior_goal.hpp"
#include "types/world.hpp"
#include "util/directed_graph.hpp"
#include "ateam_geometry/ateam_geometry.hpp"
#include "ateam_geometry/eigen_conversions.hpp"
#include "ateam_common/robot_constants.hpp"

DirectedGraph<BehaviorGoal> prepare_our_penalty_kick(const World & world)
{
  // Have one robot approach the ball
  DirectedGraph<BehaviorGoal> our_penalty_kick;
  std::optional<Ball> ball = world.get_unique_ball();
  if (!ball.has_value()) {
    return our_penalty_kick;
  }
  ateam_geometry::Point ball_location = ateam_geometry::EigenToPoint(ball.value().pos);
  // Others go 1m behind the ball in the direction towards our goal (negative direction)
  typedef CGAL::Random_points_on_segment_2<ateam_geometry::Point,
      ateam_geometry::PointCreator> linePointCreator;
  linePointCreator line_behind_ball(
    (ball_location.x() - 1.1, ball_location.y() + 5), 
    (ball_location.x() - 1.1, ball_location.y() + 5)
  );
  std::vector<ateam_geometry::Point> candidate_points;
  std::vector<ateam_geometry::Point> points_away_from_ball;
  // We can change this later to only generate the points we need
  // rather than doing them in groups of 50, that's a little silly
  while (static_cast<int>points_away_from_ball.size() < 5){
    candidate_points.reserve(50);
    std::copy_n(line_behind_ball, 50, std::back_inserter(candidate_points));
    ateam_geometry::Point previous_point = ateam_geometry::Point(-100, -100);
    for (ateam_geometry::Point candidate : candidate_points) {
      if (ateam_geometry::Segment(
            previous_point,
            candidate).squared_length() > pow(kRobotDiameter, 2))
        {
          BehaviorGoal go_to_point {
            BehaviorGoal::Type::MoveToPoint,
            BehaviorGoal::Priority::Required,
            MoveParam(ateam_geometry::PointToEigen(candidate))
          };
          points_away_from_ball.push_back(go_to_point);
          if (static_cast<int>(points_away_from_ball.size()) > 4) {
            break;
          }
    }
  }
  for (BehaviorGoal goal : points_away_from_ball){
    our_penalty_kick.add_node(goal);
  }
  return our_penalty_kick;
}

DirectedGraph<BehaviorGoal> do_our_penalty_kick(const World & world)
{
  // Have the approaching robot kick
}

DirectedGraph<BehaviorGoal> prepare_their_penalty_kick(const World & world)
{
  // Goalie goes to goal line, in the best possible position to block the ball
  // Others go 1m behind the ball in the direction towards their goal (positive direction)
}

DirectedGraph<BehaviorGoal> do_their_penalty_kick(const World & world)
{
  // Have goalie try to block the ball from the goal line
  // Continue to have other robots stay behind the ball
}

#endif  // PLAYS__PENALTY_KICK_HPP_
