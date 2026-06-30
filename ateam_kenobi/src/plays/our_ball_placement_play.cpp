// Copyright 2024 A Team
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


#include "our_ball_placement_play.hpp"
#include <angles/angles.h>
#include <algorithm>
#include <vector>
#include <ateam_common/robot_constants.hpp>
#include "core/play_helpers/available_robots.hpp"

namespace ateam_kenobi::plays
{

OurBallPlacementPlay::OurBallPlacementPlay(stp::Options stp_options)
: stp::Play(kPlayName, stp_options),
  pass_tactic_(createChild<tactics::Pass>("pass")),
  dribble_(createChild<skills::Dribble>("dribble")),
  multi_move_to_(createChild<tactics::MultiMoveTo>("multi_move_to"))
{
}

stp::PlayScore OurBallPlacementPlay::getScore(const World & world)
{
  const auto & cmd = world.referee_info.running_command;
  if (cmd == ateam_common::GameCommand::BallPlacementOurs) {
    return stp::PlayScore::Max();
  }
  return stp::PlayScore::NaN();
}

void OurBallPlacementPlay::reset()
{
  state_ = State::Placing;
  pass_tactic_.reset();
  dribble_.reset();

  spin_counter_ = 0;
  extract_pivoting_ = false;
}

std::array<std::optional<RobotCommand>, 16> OurBallPlacementPlay::runFrame(
  const World & world)
{
  std::array<std::optional<RobotCommand>, 16> maybe_motion_commands;
  auto available_robots = play_helpers::getAvailableRobots(world);

  placement_point_ = [this, &world]() {
      if (world.referee_info.designated_position.has_value()) {
        return world.referee_info.designated_position.value();
      } else {
        RCLCPP_WARN(
        getLogger(),
        "No designated position set in referee info, using ball position.");
        return world.ball.pos;
      }
    }();

  DrawKeepoutArea(world.ball.pos, placement_point_);
  getOverlays().drawCircle(
    "placement_pos", ateam_geometry::makeCircle(
      placement_point_,
      0.15), "green");

  const auto point_to_ball = world.ball.pos - placement_point_;

  const auto ball_dist = ateam_geometry::norm(point_to_ball);
  const auto ball_speed = ateam_geometry::norm(world.ball.vel);

  auto maybe_offset_vector = calculateObstacleOffset(world);

  if (world.ball.visible &&
    ball_dist < ball_placement_good_dist_ &&
    ball_speed < 0.04)
  {
    state_ = State::Done;

  // detect ball near goal/walls
  } else if (maybe_offset_vector.has_value()) {
    state_ = State::Extracting;

    // Should probably latch this point to return
    approach_point_ = world.ball.pos + maybe_offset_vector.value();
    getOverlays().drawCircle("approach_point", ateam_geometry::makeCircle(approach_point_, 0.025),
        "#0000FFFF");
  } else if (ball_dist > min_pass_distance_ && available_robots.size() > 2) {
    state_ = State::Passing;
  } else {
    state_ = State::Placing;
  }

  switch (state_) {
    case State::Extracting:
      getPlayInfo()["State"] = "Extracting";
      runExtracting(available_robots, world, maybe_motion_commands);
      break;
    case State::Passing:
      runPassing(available_robots, world, maybe_motion_commands);
      getPlayInfo()["State"] = "Passing";
      break;
    case State::Placing:
      runPlacing(available_robots, world, maybe_motion_commands);
      getPlayInfo()["State"] = "Placing";
      break;
    case State::Done:
      runDone(available_robots, world, maybe_motion_commands);
      getPlayInfo()["State"] = "Done";
  }

  // Handle all robots that haven't already been assigned a command
  available_robots.erase(
    std::remove_if(
      available_robots.begin(), available_robots.end(),
      [&maybe_motion_commands](const Robot & robot) {
        return maybe_motion_commands[robot.id].has_value();
      }),
    available_robots.end());

  const auto partition_iter = std::partition(available_robots.begin(), available_robots.end(),
      [this, &world](const Robot & robot) {
        return shouldRobotMove(world, placement_point_, robot);
    });

  const std::vector<Robot> robots_to_move(available_robots.begin(), partition_iter);
  const std::vector<Robot> robots_to_stay(partition_iter, available_robots.end());

  for(const auto & robot : robots_to_move) {
    getPlayInfo()["Robots"][std::to_string(robot.id)] = "MOVING";
  }
  for(const auto & robot : robots_to_stay) {
    getPlayInfo()["Robots"][std::to_string(robot.id)] = "-";
  }

  std::vector<ateam_geometry::Point> target_points;
  std::transform(robots_to_move.begin(), robots_to_move.end(),
    std::back_inserter(target_points),
    [this, &world](const Robot & robot) {
      return getTargetPoint(world, placement_point_, robot);
    });

  multi_move_to_.SetTargetPoints(target_points);
  multi_move_to_.SetFacePoint(world.ball.pos);
  multi_move_to_.RunFrame(robots_to_move, maybe_motion_commands);

  return maybe_motion_commands;
}

void OurBallPlacementPlay::runPassing(
  const std::vector<Robot> & available_robots, const World & world,
  std::array<std::optional<RobotCommand>,
  16> & motion_commands)
{
  if(available_robots.size() < 2) {
    RCLCPP_ERROR(getLogger(), "Cannot pass with fewer than 2 robots.");
    return;
  }

  if (prev_state_ != State::Passing) {
    pass_tactic_.reset();
  }

  std::vector<Robot> candidate_robots{available_robots.begin(), available_robots.end()};

  const auto kicker_robot = play_helpers::getClosestRobot(candidate_robots, world.ball.pos);
  play_helpers::removeRobotWithId(candidate_robots, kicker_robot.id);

  const auto receiver_robot = play_helpers::getClosestRobot(candidate_robots, placement_point_);

  getPlayInfo()["Assignments"]["Receiver"] = receiver_robot.id;
  getPlayInfo()["Assignments"]["Kicker"] = kicker_robot.id;

  // Offset the robot receiving the pass so the ball is on the placement point
  const auto ball_to_placement = placement_point_ - world.ball.pos;
  pass_tactic_.setTarget(
    placement_point_ +
    (kRobotRadius * ateam_geometry::normalize(ball_to_placement)));

  auto & kicker_command =
    *(motion_commands[kicker_robot.id] = RobotCommand{});
  auto & receiver_command =
    *(motion_commands[receiver_robot.id] = RobotCommand{});

  pass_tactic_.runFrame(world, kicker_robot, receiver_robot, kicker_command, receiver_command);

  ForwardPlayInfo(pass_tactic_);
}

void OurBallPlacementPlay::runExtracting(
  const std::vector<Robot> & available_robots, const World & world,
  std::array<std::optional<RobotCommand>,
  16> & motion_commands)
{
  const auto approach_point = approach_point_;
  const auto & extract_robot = play_helpers::getClosestRobot(available_robots, approach_point);

  getPlayInfo()["Assignments"]["Extractor"] = extract_robot.id;

  const auto robot_to_ball = (world.ball.pos - extract_robot.pos);
  const auto approach_to_ball = world.ball.pos - approach_point;

  const double approach_heading = ateam_geometry::ToHeading(approach_to_ball);

  const double robot_approach_angle_offset = ateam_geometry::ShortestAngleBetween(robot_to_ball,
      approach_to_ball);

  const bool robot_near_approach_point = ateam_geometry::norm(
    extract_robot.pos - approach_point_) < 0.05;
  const bool robot_vel_good = ateam_geometry::norm(extract_robot.vel) < 0.05;
  const bool robot_ready_to_approach = robot_near_approach_point && robot_vel_good;

  getPlayInfo()["ExtractInfo"]["robot_near_approach_point"] = robot_near_approach_point;
  getPlayInfo()["ExtractInfo"]["robot_vel_good"] = robot_vel_good;
  getPlayInfo()["ExtractInfo"]["robot_ready_to_approach"] = robot_ready_to_approach;

  const bool robot_facing_ball = abs(angles::shortest_angular_distance(approach_heading,
      extract_robot.theta)) < 0.1;
  const bool robot_near_ball = ateam_geometry::norm(robot_to_ball) <= approach_radius_;
  const bool robot_already_in_position = robot_near_ball && robot_facing_ball &&
    abs(robot_approach_angle_offset) < 0.3;

  getPlayInfo()["ExtractInfo"]["robot_facing_ball"] = robot_facing_ball;
  getPlayInfo()["ExtractInfo"]["robot_near_ball"] = robot_near_ball;
  getPlayInfo()["ExtractInfo"]["robot_approach_angle_offset"] = robot_approach_angle_offset;
  getPlayInfo()["ExtractInfo"]["robot_already_in_position"] = robot_already_in_position;

  getPlayInfo()["ExtractInfo"]["breakbeam"] = extract_robot.breakbeam_ball_detected;
  getPlayInfo()["ExtractInfo"]["breakbeam_filtered"] =
    extract_robot.breakbeam_ball_detected_filtered;

  auto motion_command = RobotCommand();
  if (extract_robot.breakbeam_ball_detected_filtered) {
    getPlayInfo()["ExtractState"] = "extracting ball";

    getPlayInfo()["extract bot approach dist"] = ateam_geometry::norm(extract_robot.pos -
        approach_point_);

    const double pivot_distance_limit = (extract_pivoting_) ? 4 * kRobotRadius : kRobotRadius;
    if (ateam_geometry::norm(extract_robot.pos - approach_point_) < pivot_distance_limit) {
      if (abs(angles::shortest_angular_distance(extract_robot.theta,
        ateam_geometry::ToHeading(placement_point_ - extract_robot.pos))) > 0.2)
      {
        getPlayInfo()["ExtractState"] = "pivoting";
        extract_pivoting_ = true;

        motion::intents::PivotPoint intent;
        // intent.planner_options.avoid_ball = false;
        intent.radius = kRobotRadius + kBallRadius;
        intent.target_x = placement_point_.x();
        intent.target_y = placement_point_.y();
        intent.inset_angle = M_PI / 2.0;
        intent.limits.angular_velocity = 1.0;
        intent.limits.angular_acceleration = 1.0;
        motion_command.motion_intent = intent;
      } else {
        // TODO(chachmu): is this really the best option?
        extract_pivoting_ = false;
        state_ = State::Placing;
      }
    } else {
      extract_pivoting_ = false;

      motion::intents::Position intent;
      intent.position = approach_point_;
      intent.heading = approach_heading;
      intent.planner_options.avoid_ball = false;
      intent.planner_options.footprint_inflation = -0.9 * kRobotRadius;
      intent.limits.linear_velocity = 0.5;
      intent.limits.linear_acceleration = 1.0;
      motion_command.motion_intent = intent;
    }
  } else if (robot_already_in_position || robot_ready_to_approach) {
    getPlayInfo()["ExtractState"] = "capturing ball";
    extract_pivoting_ = false;

    const auto point_to_ball = world.ball.pos - approach_point_;

    motion::intents::Position intent;
    intent.planner_options.avoid_ball = false;
    intent.position = approach_point_ + (approach_radius_ + kBallDiameter) *
      ateam_geometry::normalize(point_to_ball);
    intent.limits.linear_velocity = 0.5;
    intent.limits.linear_acceleration = 0.5;
    intent.heading = approach_heading;
    motion_command.motion_intent = intent;
  } else {
    getPlayInfo()["ExtractState"] = "moving to approach point";
    motion::intents::Position intent;
    intent.planner_options.avoid_ball = false;
    intent.position = approach_point_;
    intent.heading = approach_heading;
    // intent.limits.linear_velocity = 2.0;
    // intent.limits.linear_acceleration = 2.0;
    motion_command.motion_intent = intent;
  }

  const bool should_dribble = extract_robot.breakbeam_ball_detected_filtered ||
    robot_ready_to_approach || robot_already_in_position;
  if (should_dribble) {
    motion_command.dribbler_setpoint = 0.025;
  }

  // Check if motion intent has a planner options field and set the boundary footprint inflation very small

  std::visit([](auto & intent) {
      if constexpr (motion::has_planner_options<decltype(intent)>) {
        intent.planner_options.boundary_footprint_inflation = -0.1;
      }
  }, motion_command.motion_intent);

  motion_commands[extract_robot.id] = motion_command;
}

void OurBallPlacementPlay::runPlacing(
  const std::vector<Robot> & available_robots, const World & world,
  std::array<std::optional<RobotCommand>,
  16> & motion_commands)
{

  if (prev_state_ != State::Placing) {
    dribble_.reset();
  }
  dribble_.setTarget(placement_point_);

  const auto & place_robot = play_helpers::getClosestRobot(available_robots,
      dribble_.getAssignmentPoint(world));
  getPlayInfo()["Assignments"]["Placer"] = place_robot.id;

  motion_commands[place_robot.id] = dribble_.runFrame(world, place_robot);
  ForwardPlayInfo(dribble_);
}

void OurBallPlacementPlay::runDone(
  const std::vector<Robot> & available_robots, const World & world,
  std::array<std::optional<RobotCommand>,
  16> & motion_commands)
{
  // TODO(chachmu): Might need to add a delay for this so the dribbler slows down

  auto byDistToBall = [&world](const Robot & lhs, const Robot & rhs) {
      return CGAL::compare_distance_to_point(world.ball.pos, lhs.pos, rhs.pos) == CGAL::SMALLER;
    };

  const auto place_robot = *std::min_element(
    available_robots.begin(),
    available_robots.end(), byDistToBall);

  getPlayInfo()["Assignments"]["Placer"] = place_robot.id;

  const auto ball_to_robot = place_robot.pos - world.ball.pos;

  double rules_backoff_distance = 0.5;
  if (world.referee_info.next_command.has_value()) {
    switch(world.referee_info.next_command.value()) {
      case ateam_common::GameCommand::DirectFreeOurs:
        rules_backoff_distance = 0.05;
        break;
      case ateam_common::GameCommand::ForceStart:
      default:
        rules_backoff_distance = 0.5;
    }
  }

  const auto final_position = world.ball.pos +
    ((rules_backoff_distance + 1.5 * kRobotDiameter) * ateam_geometry::normalize(ball_to_robot));

  getPlayInfo()["requested_backoff_distance"] = rules_backoff_distance + 1.5 * kRobotDiameter;
  getPlayInfo()["robot_backoff_distance"] = ateam_geometry::norm(final_position - place_robot.pos);

  RobotCommand command;
  if (spin_counter_ >= 3 || ateam_geometry::norm(final_position - place_robot.pos) > kRobotRadius) {
    motion::intents::PositionFacing intent;
    intent.position = final_position;
    intent.face_target = world.ball.pos;
    command.motion_intent = intent;
  } else {
    const double to_ball = ateam_geometry::ToHeading(world.ball.pos - place_robot.pos);
    const double target_heading = to_ball + (spin_counter_ + 1) * (2 * M_PI / 3.0);

    motion::intents::Position intent;
    intent.position = final_position;
    intent.heading = target_heading;
    command.motion_intent = intent;

    if (abs(angles::shortest_angular_distance(target_heading, place_robot.theta)) < M_PI / 4.0) {
      spin_counter_++;
    }
  }

  motion_commands[place_robot.id] = command;
}

void OurBallPlacementPlay::DrawKeepoutArea(
  const ateam_geometry::Point & ball_pos,
  const ateam_geometry::Point & placement_point)
{
  const auto point_to_ball = ball_pos - placement_point;
  const auto angle = std::atan2(point_to_ball.y(), point_to_ball.x());

  auto & overlays = getOverlays();

  const auto keepout_radius = 0.5;
  const ateam_geometry::Vector pos_offset{keepout_radius * std::cos(angle + M_PI_2),
    keepout_radius * std::sin(angle + M_PI_2)};
  const ateam_geometry::Vector neg_offset{keepout_radius * std::cos(angle - M_PI_2),
    keepout_radius * std::sin(angle - M_PI_2)};

  const ateam_geometry::Arc ball_side_arc{ball_pos, keepout_radius, neg_offset.direction(),
    pos_offset.direction()};
  const ateam_geometry::Arc point_side_arc{placement_point, keepout_radius, pos_offset.direction(),
    neg_offset.direction()};
  const ateam_geometry::Segment pos_segment{placement_point + pos_offset, ball_pos + pos_offset};
  const ateam_geometry::Segment neg_segment{placement_point + neg_offset, ball_pos + neg_offset};


  overlays.drawArc("placement_avoid_ball_arc", ball_side_arc, "Cyan");
  overlays.drawArc("placement_avoid_place_arc", point_side_arc, "Cyan");
  overlays.drawLine("placement_avoid_pos_line", {pos_segment.source(), pos_segment.target()},
      "Cyan");
  overlays.drawLine("placement_avoid_neg_line", {neg_segment.source(), neg_segment.target()},
      "Cyan");
}

std::optional<ateam_geometry::Vector> OurBallPlacementPlay::calculateObstacleOffset(
  const World & world)
{
  double obstacle_offset_x = 0;
  double obstacle_offset_y = 0;

  // Handle ball inside goal
  bool is_inside_goal_x = abs(world.ball.pos.x()) > world.field.field_length / 2;
  bool is_between_goalposts = abs(world.ball.pos.y()) < world.field.goal_width / 2;
  bool is_inside_goal = is_inside_goal_x && is_between_goalposts;

  if (is_inside_goal) {
    bool is_near_back_goal_x = abs(world.ball.pos.x()) + ball_distance_from_obstacle_ >
      (world.field.field_length / 2) + world.field.goal_depth;
    bool is_near_inside_goalposts_y = abs(world.ball.pos.y()) + ball_distance_from_obstacle_ >
      world.field.goal_width / 2;

    if (is_near_back_goal_x) {
      obstacle_offset_x = (world.ball.pos.x() > 0) ? -1.0 : 1.0;
    }
    if (is_near_inside_goalposts_y) {
      obstacle_offset_y = (world.ball.pos.y() > 0) ? -1.0 : 1.0;
    }

    auto offset_vector = approach_radius_ *
      ateam_geometry::normalize(ateam_geometry::Vector(obstacle_offset_x, obstacle_offset_y));
    return std::make_optional(offset_vector);
  }

  // Handle ball near outside goalposts
  bool is_near_outside_goalposts_x = abs(world.ball.pos.x()) + ball_distance_from_obstacle_ >
    world.field.field_length / 2;
  bool is_near_outside_goalposts_y = abs(world.ball.pos.y()) - ball_distance_from_obstacle_ <
    (world.field.goal_width / 2) + 0.02;
  bool is_near_outside_goalposts = is_near_outside_goalposts_x && is_near_outside_goalposts_y;

  if (is_near_outside_goalposts) {
    if (is_near_outside_goalposts_x) {
      obstacle_offset_x = (world.ball.pos.x() > 0) ? -1.0 : 1.0;
    }
    if (is_near_outside_goalposts_y) {
      obstacle_offset_y = (world.ball.pos.y() > 0) ? 1.0 : -1.0;
    }

    auto offset_vector = approach_radius_ *
      ateam_geometry::normalize(ateam_geometry::Vector(obstacle_offset_x, obstacle_offset_y));
    return std::make_optional(offset_vector);
  }

  // Handle ball near walls
  bool is_near_x_walls = abs(world.ball.pos.x()) + ball_distance_from_obstacle_ >
    (world.field.field_length / 2) + world.field.boundary_width;
  bool is_near_y_walls = abs(world.ball.pos.y()) + ball_distance_from_obstacle_ >
    (world.field.field_width / 2) + world.field.boundary_width;
  bool is_near_wall = is_near_x_walls || is_near_y_walls;

  if (is_near_wall) {
    if (is_near_x_walls) {
      obstacle_offset_x = (world.ball.pos.x() > 0) ? -1.0 : 1.0;
    }
    if (is_near_y_walls) {
      obstacle_offset_y = (world.ball.pos.y() > 0) ? -1.0 : 1.0;
    }

    auto offset_vector = approach_radius_ *
      ateam_geometry::normalize(ateam_geometry::Vector(obstacle_offset_x, obstacle_offset_y));
    return std::make_optional(offset_vector);
  }

  // No obstacles to account for
  return std::nullopt;
}

bool OurBallPlacementPlay::shouldRobotMove(
  const World & world,
  const ateam_geometry::Point & placement_point, const Robot & robot)
{
  const auto placement_segment = ateam_geometry::Segment(placement_point, world.ball.pos);
  const auto nearest_point = ateam_geometry::nearestPointOnSegment(placement_segment, robot.pos);
  return ateam_geometry::norm(robot.pos - nearest_point) < 0.6 + kRobotRadius;
}

ateam_geometry::Point OurBallPlacementPlay::getTargetPoint(
  const World & world,
  const ateam_geometry::Point & placement_point, const Robot & robot)
{
  const auto placement_segment = ateam_geometry::Segment(placement_point, world.ball.pos);
  const auto nearest_point = ateam_geometry::nearestPointOnSegment(placement_segment, robot.pos);

  const auto point_to_ball = world.ball.pos - placement_point;
  const auto angle = std::atan2(point_to_ball.y(), point_to_ball.x());

  auto target_position = nearest_point +
    0.7 * ateam_geometry::Vector(std::cos(angle + M_PI / 2), std::sin(angle + M_PI / 2));

  const auto alternate_position = nearest_point +
    0.7 * ateam_geometry::Vector(std::cos(angle - M_PI / 2), std::sin(angle - M_PI / 2));

  const auto offset = kRobotRadius * 0.95;
  const auto x_bound = (world.field.field_length / 2.0) + world.field.boundary_width - offset;
  const auto y_bound = (world.field.field_width / 2.0) + world.field.boundary_width - offset;
  ateam_geometry::Rectangle pathable_region(ateam_geometry::Point(-x_bound, -y_bound),
    ateam_geometry::Point(x_bound, y_bound));

  if (!CGAL::do_intersect(target_position, pathable_region)) {
    target_position = alternate_position;
  } else if (!CGAL::do_intersect(alternate_position, pathable_region)) {
        // Stick with target_position
  } else {
        // Use the shorter path
    if (ateam_geometry::norm(target_position - robot.pos) >
      ateam_geometry::norm(alternate_position - robot.pos))
    {
      target_position = alternate_position;
    }
  }
  return target_position;
}

}  // namespace ateam_kenobi::plays
