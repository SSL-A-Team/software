// Copyright 2021 A Team
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


#include "goalie.hpp"
#include <algorithm>
#include <limits>
#include <ateam_common/robot_constants.hpp>
#include <ateam_geometry/nearest_point.hpp>
#include "play_helpers/window_evaluation.hpp"
#include "play_helpers/available_robots.hpp"

namespace ateam_kenobi::skills
{

Goalie::Goalie(stp::Options stp_options)
: stp::Skill(stp_options),
  easy_move_to_(createChild<play_helpers::EasyMoveTo>("EasyMoveTo")),
  line_kick_(createChild<skills::LineKick>("LineKick"))
{
  reset();
}

void Goalie::reset()
{
  easy_move_to_.reset();
  path_planning::PlannerOptions planner_options;
  planner_options.avoid_ball = false;
  planner_options.use_default_obstacles = false;
  easy_move_to_.setPlannerOptions(planner_options);
}

void Goalie::runFrame(
  const World & world,
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
  16> & motion_commands)
{
  const auto robot_id = world.referee_info.our_goalie_id;
  const auto & robot = world.our_robots.at(robot_id);
  if (robot.is_valid()) {
    // Assigned robot is not visible
    return;
  }

  ateam_msgs::msg::RobotMotionCommand motion_command;

  if (isBallHeadedTowardsGoal(world)) {
    motion_command = runBlockBall(world, robot);
  } else if (doesOpponentHavePossesion(world)) {
    motion_command = runBlockShot(world, robot);
  } else if (isBallInDefenseArea(world)) {
    motion_command = runClearBall(world, robot);
  } else {
    motion_command = runDefaultBehavior(world, robot);
  }

  motion_commands[robot_id] = motion_command;
}

bool Goalie::doesOpponentHavePossesion(const World & world)
{
  const auto & ball_pos = world.ball.pos;
  std::array<double, 16> distances;
  std::transform(
    world.their_robots.begin(), world.their_robots.end(), distances.begin(),
    [&ball_pos](const auto & robot) {
      if (!robot.is_valid()) {
        return std::numeric_limits<double>::infinity();
      }
      return CGAL::approximate_sqrt(CGAL::squared_distance(ball_pos, robot.pos));
    });
  const auto min_distance = *std::min_element(distances.begin(), distances.end());
  return min_distance < kRobotRadius + possesion_threshold_;
}

bool Goalie::isBallHeadedTowardsGoal(const World & world)
{
  if (world.ball.vel.x() >= -0.1) {
    return false;
  }

  const ateam_geometry::Ray ball_vel_ray(world.ball.pos, world.ball.vel);

  const auto goal_inflation = 1.1;

  const ateam_geometry::Segment goal_segment(
    ateam_geometry::Point(
      -world.field.field_length / 2,
      -(world.field.goal_width * goal_inflation) / 2),
    ateam_geometry::Point(
      -world.field.field_length / 2,
      (world.field.goal_width * goal_inflation) / 2)
  );

  return CGAL::do_intersect(ball_vel_ray, goal_segment);
}

bool Goalie::isBallInDefenseArea(const World & world)
{
  ateam_geometry::Rectangle our_defense_area(
    *CGAL::top_vertex_2(
      world.field.ours.defense_area_corners.begin(),
      world.field.ours.defense_area_corners.end()),
    *CGAL::bottom_vertex_2(
      world.field.ours.defense_area_corners.begin(),
      world.field.ours.defense_area_corners.end()));

  return CGAL::do_intersect(world.ball.pos, our_defense_area);
}

ateam_msgs::msg::RobotMotionCommand Goalie::runDefaultBehavior(
  const World & world,
  const Robot & goalie)
{
  const ateam_geometry::Segment goalie_line(
    ateam_geometry::Point(-(world.field.field_length / 2) + 0.25, world.field.goal_width / 2),
    ateam_geometry::Point(-(world.field.field_length / 2) + 0.25, -world.field.goal_width / 2));

  easy_move_to_.setTargetPosition(
    ateam_geometry::nearestPointOnSegment(
      goalie_line,
      world.ball.pos));
  easy_move_to_.face_point(world.ball.pos);
  return easy_move_to_.runFrame(goalie, world);
}

ateam_msgs::msg::RobotMotionCommand Goalie::runBlockShot(const World & world, const Robot & goalie)
{
  const auto & ball_pos = world.ball.pos;
  std::array<double, 16> distances;
  std::transform(
    world.their_robots.begin(), world.their_robots.end(), distances.begin(),
    [&ball_pos](const auto & robot) {
      if (!robot.is_valid()) {
        return std::numeric_limits<double>::infinity();
      }
      return CGAL::approximate_sqrt(CGAL::squared_distance(ball_pos, robot.pos));
    });
  const auto min_distance_iter = std::min_element(distances.begin(), distances.end());
  if (std::isinf(*min_distance_iter)) {
    // Should only be true if there are no opponent robots
    return runDefaultBehavior(world, goalie);
  }
  const auto opponent_id = std::distance(distances.begin(), min_distance_iter);
  const auto opponent_pos = world.their_robots[opponent_id].pos;
  const auto opponent_ball_vector = world.ball.pos - opponent_pos;

  const ateam_geometry::Ray shot_ray(opponent_pos, opponent_ball_vector);

  const auto goalie_line_x = -(world.field.field_length / 2) + kRobotRadius;

  const ateam_geometry::Segment goalie_line_extended(
    ateam_geometry::Point(goalie_line_x, world.field.field_width / 2),
    ateam_geometry::Point(goalie_line_x, -world.field.field_width / 2));

  const ateam_geometry::Segment goalie_line(
    ateam_geometry::Point(goalie_line_x, world.field.goal_width / 2),
    ateam_geometry::Point(goalie_line_x, -world.field.goal_width / 2));

  const auto maybe_intersection = CGAL::intersection(shot_ray, goalie_line_extended);

  if (!maybe_intersection) {
    return runDefaultBehavior(world, goalie);
  }

  ateam_geometry::Point shot_point_on_extended_goalie_line;

  if (const auto * point = boost::get<ateam_geometry::Point>(&*maybe_intersection)) {
    shot_point_on_extended_goalie_line = *point;
  } else if (const auto * segment = boost::get<ateam_geometry::Segment>(&*maybe_intersection)) {
    shot_point_on_extended_goalie_line = segment->source();
  }

  easy_move_to_.setTargetPosition(
    ateam_geometry::nearestPointOnSegment(
      goalie_line,
      shot_point_on_extended_goalie_line));
  easy_move_to_.face_point(world.ball.pos);

  return easy_move_to_.runFrame(goalie, world);
}

ateam_msgs::msg::RobotMotionCommand Goalie::runBlockBall(const World & world, const Robot & goalie)
{
  const ateam_geometry::Ray ball_vel_ray(world.ball.pos, world.ball.vel);

  const auto goalie_line_x = -(world.field.field_length / 2) + kRobotRadius;

  const ateam_geometry::Segment goalie_line(
    ateam_geometry::Point(goalie_line_x, world.field.goal_width / 2),
    ateam_geometry::Point(goalie_line_x, -world.field.goal_width / 2));

  const auto maybe_intersection = CGAL::intersection(ball_vel_ray, goalie_line);

  if (!maybe_intersection) {
    return runDefaultBehavior(world, goalie);
  }

  ateam_geometry::Point target_point;

  if (const auto * point = boost::get<ateam_geometry::Point>(&*maybe_intersection)) {
    target_point = *point;
  } else if (const auto * segment = boost::get<ateam_geometry::Segment>(&*maybe_intersection)) {
    target_point = segment->source();
  }

  easy_move_to_.setTargetPosition(target_point);
  easy_move_to_.face_point(world.ball.pos);

  return easy_move_to_.runFrame(goalie, world);
}

ateam_msgs::msg::RobotMotionCommand Goalie::runClearBall(const World & world, const Robot & goalie)
{
  const ateam_geometry::Segment target_seg(ateam_geometry::Point(0, -world.field.field_width / 2),
    ateam_geometry::Point(0, world.field.field_width / 2));

  auto robots = play_helpers::getVisibleRobots(world.our_robots);
  play_helpers::removeRobotWithId(robots, goalie.id);
  std::ranges::copy(play_helpers::getVisibleRobots(world.their_robots), std::back_inserter(robots));

  const auto windows = play_helpers::window_evaluation::getWindows(
    target_seg, world.ball.pos,
    robots);
  play_helpers::window_evaluation::drawWindows(
    windows, world.ball.pos, getOverlays().getChild(
      "windows"));
  const auto largest_window = play_helpers::window_evaluation::getLargestWindow(windows);

  ateam_geometry::Point target_point;

  if (largest_window) {
    target_point = CGAL::midpoint(*largest_window);
  } else {
    // If no window exists, just kick away from the goal
    const ateam_geometry::Point goal_center(-world.field.field_length / 2, 0);
    const auto kick_vector = world.ball.pos - goal_center;
    target_point = world.ball.pos + (kick_vector * 3);
  }

  line_kick_.setTargetPoint(target_point);

  return line_kick_.runFrame(world, goalie);
}

}  // namespace ateam_kenobi::skills
