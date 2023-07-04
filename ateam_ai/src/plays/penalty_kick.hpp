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
#include "plays/shoot.hpp"
#include "ateam_geometry/ateam_geometry.hpp"
#include "ateam_geometry/eigen_conversions.hpp"
#include "ateam_geometry/nearest_points.hpp"
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
    ateam_geometry::Point(ball_location.x() - 1.3, ball_location.y() + 5), 
    ateam_geometry::Point(ball_location.x() - 1.3, ball_location.y() + 5)
  );
  std::vector<ateam_geometry::Point> candidate_points;
  std::vector<BehaviorGoal> points_away_from_ball;
  // We can change this later to only generate the points we need
  // rather than doing them in groups of 50, that's a little silly
  while (static_cast<int>(points_away_from_ball.size()) < 5){
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
}

DirectedGraph<BehaviorGoal> do_our_penalty_kick(const World & world){
  // Kick the ball
  DirectedGraph<BehaviorGoal> our_penalty_kick = generate_basic_shoot(world);
  // Have everyone else halt
  for (int i = 0; i < 5; i++) {
    BehaviorGoal halt {
      BehaviorGoal::Type::Halt,
      BehaviorGoal::Priority::Required,
      HaltParam()
    };
    our_penalty_kick.add_node(halt);
  }
  return our_penalty_kick;
}

DirectedGraph<BehaviorGoal> prepare_their_penalty_kick(const World & world){
  DirectedGraph<BehaviorGoal> their_penalty_kick;
  std::optional<Ball> ball = world.get_unique_ball();
  if (!ball.has_value()) {
    return their_penalty_kick;
  }
  ateam_geometry::Point ball_location = ateam_geometry::EigenToPoint(ball.value().pos);
  // Goalie goes to goal line, in the best possible position to block the ball
  ateam_geometry::Segment goal_line = ateam_geometry::Segment(
    ateam_geometry::Point(-world.field.field_width/2,500),
    ateam_geometry::Point(-world.field.field_width/2,-500)
  );
  ateam_geometry::Point goalie_point = ateam_geometry::NearestPointOnSegment(goal_line, ball_location);
  their_penalty_kick.add_node(
    BehaviorGoal(
    BehaviorGoal::Type::MoveToPoint,
    BehaviorGoal::Priority::Reserved,
    MoveParam(ateam_geometry::PointToEigen(goalie_point)),
    world.referee_info.our_goalie_id
  ));
  // Others go 1m behind the ball in the direction towards their goal (positive direction)
  typedef CGAL::Random_points_on_segment_2<ateam_geometry::Point,
      ateam_geometry::PointCreator> linePointCreator;
  linePointCreator line_behind_ball(
    ateam_geometry::Point(ball_location.x() + 1.3, ball_location.y() + 5), 
    ateam_geometry::Point(ball_location.x() + 1.3, ball_location.y() + 5)
  );
  std::vector<ateam_geometry::Point> candidate_points;
  std::vector<BehaviorGoal> points_away_from_ball;
  // We can change this later to only generate the points we need
  // rather than doing them in groups of 50, that's a little silly
  while (static_cast<int>(points_away_from_ball.size()) < 5){
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
    their_penalty_kick.add_node(goal);
  }
  return their_penalty_kick;
  }
}

DirectedGraph<BehaviorGoal> do_their_penalty_kick(const World & world)
{
  // Have goalie try to block the ball from the goal line
  DirectedGraph<BehaviorGoal> their_penalty_kick;
  std::optional<Ball> ball = world.get_unique_ball();
  if (!ball.has_value()) {
    return their_penalty_kick;
  }
  ateam_geometry::Point ball_location = ateam_geometry::EigenToPoint(ball.value().pos);
  // Goalie goes to goal line, in the best possible position to block the ball
  ateam_geometry::Segment goal_line = ateam_geometry::Segment(
      ateam_geometry::Point(-world.field.field_width/2,500),
      ateam_geometry::Point(-world.field.field_width/2,-500)
  );
  ateam_geometry::Point goalie_point = ateam_geometry::NearestPointOnSegment(goal_line, ball_location);
  their_penalty_kick.add_node(
    BehaviorGoal(
    BehaviorGoal::Type::MoveToPoint,
    BehaviorGoal::Priority::Reserved,
    MoveParam(ateam_geometry::PointToEigen(goalie_point)),
    world.referee_info.our_goalie_id
  ));
  // Continue to have other robots stay behind the ball
  for (int i = 0; i < 5; i++) {
    // Generate a required halt for every robot on our team
    BehaviorGoal halt {
      BehaviorGoal::Type::Halt,
      BehaviorGoal::Priority::Required,
      HaltParam()
    };
    their_penalty_kick.add_node(halt);
  }
  return their_penalty_kick;
}

#endif  // PLAYS__PENALTY_KICK_HPP_
