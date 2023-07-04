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
#include <vector>
#include <optional>

#include "util/directed_graph.hpp"
#include "types/world.hpp"
#include "types/behavior_goal.hpp"
#include "ateam_geometry/ateam_geometry.hpp"
#include "ateam_geometry/nearest_points.hpp"
#include "plays/play_helpers.hpp"

std::vector<BehaviorGoal> get_defense_behavior_goals(
  const World & world,
  const int & num_defenders)
{
  std::vector<BehaviorGoal> defenders;
  std::optional<Ball> ball = world.get_unique_ball();
  if (!ball.has_value()) {
    return defenders;
  }
  // Get line between the ball and the goal
  ateam_geometry::Point ball_location = ateam_geometry::EigenToPoint(ball.value().pos);
  ateam_geometry::Point middle_of_our_goal {-4.5, 0};
  // Object to generate candidate points on this line to block
  std::vector<ateam_geometry::Point> candidate_points;
  typedef CGAL::Random_points_on_segment_2<ateam_geometry::Point,
      ateam_geometry::PointCreator> linePointCreator;
  linePointCreator line_to_goal(ball_location, middle_of_our_goal);
  // Get two defenders
  while (static_cast<int>(defenders.size()) < num_defenders) {
    candidate_points.reserve(50);
    std::copy_n(line_to_goal, 50, std::back_inserter(candidate_points));
    // Remove any that will cause us to be out of bounds
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
          defenders.push_back(go_to_point);
          if (static_cast<int>(defenders.size()) > num_defenders - 1) {
            break;
          }
        }
      }
    }
  }
  // Have robots go to these points
  return defenders;
}

BehaviorGoal get_goalie_behavior_goal(const World & world)
{
  // Line that is 0.5 m from the defense area in all directions
  ateam_geometry::Segment goalie_line = ateam_geometry::Segment(
    ateam_geometry::Point(
      -4,
      0.5), ateam_geometry::Point(
      -4, -0.5));

  // Get the ball location
  std::optional<Ball> ball = world.get_unique_ball();
  // Since we will always want a goalie somewhere, we just choose
  // a point for it to intercept rather than returning empty behaviors
  ateam_geometry::Point ball_location;
  if (!ball.has_value()) {
    ball_location = ateam_geometry::Point(0, 0);
  } else {
    ball_location = ateam_geometry::EigenToPoint(ball.value().pos);
  }
  // Get the point on the goalie line that is closest to the ball
  ateam_geometry::Point _goalie_point = ateam_geometry::NearestPointOnSegment(
    goalie_line,
    ball_location);

  // Have the goalie defend the goal by going to that point
  BehaviorGoal goalie(
    BehaviorGoal::Type::MoveToPoint,
    BehaviorGoal::Priority::Reserved,
    MoveParam(ateam_geometry::PointToEigen(_goalie_point)),
    world.referee_info.our_goalie_id
  );

  return goalie;
}

  // Go to the middle of the goalie area
  BehaviorGoal goalie = get_goalie_behavior_goal(world);
  defense_graph.add_node(goalie);

  std::vector<BehaviorGoal> defenders = get_defense_behavior_goals(world, 2);
  for (BehaviorGoal defender : defenders) {
    defense_graph.add_node(defender);
  }
  return defense_graph;
}

BehaviorGoal get_goalie_behavior_goal(const World & world)
{
  // Line that is 0.5 m from the defense area in all directions
  ateam_geometry::Segment goalie_line = ateam_geometry::Segment(
    ateam_geometry::Point(
      -4,
      0.5), ateam_geometry::Point(
      -4, -0.5));

  // Get the ball location
  std::optional<Eigen::Vector2d> ball_location = world.get_unique_ball();
  if (!ball_location.has_value()) {
    // Since we will always want a goalie somewhere, we just choose
    // a point for it to intercept rather than returning empty behaviors
    ball_location = ateam_geometry::Point(0, 0);
  }

  // Get the point on the goalie line that is closest to the ball
  ateam_geometry::Point _goalie_point = goalie_line.projection(ball_location);

  // Have the goalie defend the goal by going to that point
  return BehaviorGoal goalie {
    BehaviorGoal::Type::MoveToPoint,
    BehaviorGoal::Priority::Reserved,
    MoveParam(_goalie_point),
    reserved_robot_id = world.referee_info.our_goalie_id
  };
}

std::vector<BehaviorGoal> get_defense_behavior_goals(
  const World & world,
  const int & num_defenders)
{
  // TODO(Christian): Replace the below with the kRobotDiameter constant
  double robot_diameter = 0.18;
  std::vector<BehaviorGoal> defenders;
  std::optional<Eigen::Vector2d> ball_location = world.get_unique_ball();
  // Get line between the ball and the goal
  if (!ball_location.has_value()) {
    return defenders;
  }
  ateam_geometry::Point ball = ateam_geometry::EigenToPoint(ball_location);
  ateam_geometry::Point middle_of_our_goal = {-4.5, 0};
  ateam_geometry::Segment block_line = ateam_geometry::Segment(ball, middle_of_our_goal);
  // Object to generate candidate points on this line to block
  std::vector<ateam_geometry::Point> candidate_points;
  Random_points_on_segment_2<ateam_geometry::Point, ateam_geometry::PointCreator> linePointCreator;
  linePointCreator line_to_goal(ball, middle_of_our_goal);
  // Get two defenders
  while (defenders.size() < num_defenders) {
    candidate_points.reserve(50);
    std::copy_n(line_to_goal, 50, std::back_inserter(candidate_points));
    // Remove any that will cause us to be out of bounds
    ateam_geometry::Point previous_point = ateam_geometry::Point(-100, -100);
    for (ateam_geometry::Point candidate : candidate_points) {
      if (is_point_in_bounds(candidate)) {
        if (ateam_geometry::Segment(
            previous_point,
            candidate).squared_length() > pow(robot_diameter, 2))
        {
          BehaviorGoal go_to_point {
            BehaviorGoal::Type::MoveToPoint,
            BehaviorGoal::Priority::Required,
            MoveParam(ateam_geometry::PointToEigen(candidate))
          };
          defenders.push_back(go_to_point);
          if (defenders.size() > num_defenders - 1) {
            break;
          }
        }
      }
    }
  }
  // Have robots go to these points
  return defenders;
}

#endif  // PLAYS__DEFENSE_HPP_
