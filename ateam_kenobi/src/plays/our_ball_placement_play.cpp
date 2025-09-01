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
#include <vector>
#include <ateam_common/robot_constants.hpp>
#include "core/play_helpers/available_robots.hpp"

namespace ateam_kenobi::plays
{

OurBallPlacementPlay::OurBallPlacementPlay(stp::Options stp_options)
: stp::Play(kPlayName, stp_options),
  pass_tactic_(createChild<tactics::Pass>("pass")),
  dribble_(createChild<skills::Dribble>("dribble"))
{
  createIndexedChildren<play_helpers::EasyMoveTo>(easy_move_tos_, "EasyMoveTo");
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
  for (auto & emt : easy_move_tos_) {
    emt.reset();
  }
}

std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> OurBallPlacementPlay::runFrame(
  const World & world)
{
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> maybe_motion_commands;

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

  const auto point_to_ball = world.ball.pos - placement_point_;
  const auto angle = std::atan2(point_to_ball.y(), point_to_ball.x());

  const auto ball_dist = ateam_geometry::norm(point_to_ball);
  const auto ball_speed = ateam_geometry::norm(world.ball.vel);

  getOverlays().drawCircle(
    "placement_pos", ateam_geometry::makeCircle(
      placement_point_,
      0.15), "green");

  auto maybe_offset_vector = calculateObstacleOffset(world);

  const bool use_extract = true;
  const auto should_extract = use_extract && world.ball.visible && ball_dist > 0.2 &&
    maybe_offset_vector.has_value();

  if (state_ != State::Placing &&
    world.ball.visible &&
    ball_dist < 0.12 &&
    ball_speed < 0.04)
  {
    state_ = State::Done;

  // detect ball near goal/walls
  } else if (should_extract) {
    approach_point_ = world.ball.pos + maybe_offset_vector.value();
    state_ = State::Extracting;
    getOverlays().drawCircle("approach_point", ateam_geometry::makeCircle(approach_point_, 0.025),
        "#0000FFFF");
  }

  const bool see_ball_clear_of_obstacle = world.ball.visible && !maybe_offset_vector.has_value();

  switch (state_) {
    case State::Extracting:
      // ball is clear of obstacles
      if (see_ball_clear_of_obstacle) {
        state_ = State::Placing;
        break;
      }

      getPlayInfo()["State"] = "Extracting";
      runExtracting(available_robots, world, maybe_motion_commands);
      break;
    case State::Passing:
      if (pass_tactic_.isDone() ||
        (ball_dist < 1.0 && ball_speed < 0.05) ||
        available_robots.size() < 2)
      {
        state_ = State::Placing;
        break;
      }
      runPassing(available_robots, world, maybe_motion_commands);
      getPlayInfo()["State"] = "Passing";
      break;
    case State::Placing:
      // Ball must be placed in a 0.15m radius
      // if (ball_dist < 0.13 && ball_speed < 0.08) {
      if (dribble_.isDone() ||
        (false && ball_dist < 0.08 && ball_speed < 0.04))
      {
        state_ = State::Done;

        // Can try to pass if the ball is far away and we have enough robots
      } else if (ball_dist > 1.1 && available_robots.size() >= 2) {
        pass_tactic_.reset();
        state_ = State::Passing;
      }
      runPlacing(available_robots, world, maybe_motion_commands);
      getPlayInfo()["State"] = "Placing";
      break;
    case State::Done:
      if (ball_dist > 0.14) {
        dribble_.reset();
        state_ = State::Placing;
      }
      runDone(available_robots, world, maybe_motion_commands);
      getPlayInfo()["State"] = "Done";
  }

  for (auto ind = 0ul; ind < available_robots.size(); ++ind) {
    const auto & robot = available_robots[ind];
    auto & motion_command = maybe_motion_commands[robot.id];

    if (!motion_command) {
      auto & emt = easy_move_tos_[robot.id];

      const auto placement_segment = ateam_geometry::Segment(placement_point_, world.ball.pos);
      const auto nearest_point =
        ateam_geometry::nearestPointOnSegment(placement_segment, robot.pos);

      ateam_geometry::Point target_position = robot.pos;
      if (ateam_geometry::norm(robot.pos - nearest_point) < 0.6 + kRobotRadius) {

        target_position = nearest_point +
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

        emt.setTargetPosition(target_position);
        emt.face_point(world.ball.pos);
        maybe_motion_commands[robot.id] = emt.runFrame(robot, world);

        getPlayInfo()["Robots"][std::to_string(robot.id)] = "MOVING";
      } else {
        getPlayInfo()["Robots"][std::to_string(robot.id)] = "-";
      }
    }
  }

  return maybe_motion_commands;
}

void OurBallPlacementPlay::runPassing(
  const std::vector<Robot> & available_robots, const World & world,
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
  16> & motion_commands)
{
  if(available_robots.size() < 2) {
    RCLCPP_ERROR(getLogger(), "Cannot pass with fewer than 2 robots.");
    return;
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
    *(motion_commands[kicker_robot.id] = ateam_msgs::msg::RobotMotionCommand{});
  auto & receiver_command =
    *(motion_commands[receiver_robot.id] = ateam_msgs::msg::RobotMotionCommand{});

  pass_tactic_.runFrame(world, kicker_robot, receiver_robot, kicker_command, receiver_command);

  ForwardPlayInfo(pass_tactic_);
}

void OurBallPlacementPlay::runExtracting(
  const std::vector<Robot> & available_robots, const World & world,
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
  16> & motion_commands)
{
  const auto approach_point = approach_point_;
  auto byDistToApproachPoint = [&approach_point](const Robot & lhs, const Robot & rhs) {
      return CGAL::compare_distance_to_point(approach_point, lhs.pos, rhs.pos) == CGAL::SMALLER;
    };

  const auto & extract_robot = *std::min_element(
    available_robots.begin(),
    available_robots.end(), byDistToApproachPoint);

  getPlayInfo()["Assignments"]["Extractor"] = extract_robot.id;

  auto & emt = easy_move_tos_[extract_robot.id];

  const auto robot_to_ball = (world.ball.pos - extract_robot.pos);
  const auto ball_to_approach = approach_point_ - world.ball.pos;

  // const double ball_to_approach_angle = std::atan2(ball_to_approach.y(), ball_to_approach.x());
  const double robot_approach_angle_offset = std::acos((-robot_to_ball * ball_to_approach) /
      (ateam_geometry::norm(robot_to_ball) * ateam_geometry::norm(ball_to_approach))
  );
  const double robot_to_ball_angle = std::atan2(robot_to_ball.y(), robot_to_ball.x());

  const bool robot_near_approach_point = ateam_geometry::norm(
    extract_robot.pos - approach_point_
    ) < 0.05;
  const bool robot_facing_ball = abs(robot_to_ball_angle - extract_robot.theta) < 0.1;
  const bool robot_near_ball = ateam_geometry::norm(robot_to_ball) <= approach_radius_;
  const bool robot_already_in_position = robot_near_ball && robot_facing_ball &&
    abs(robot_approach_angle_offset) < 0.3;

  auto motion_command = ateam_msgs::msg::RobotMotionCommand();
  if (extract_robot.breakbeam_ball_detected_filtered) {
    getPlayInfo()["ExtractState"] = "extracting ball";

    getPlayInfo()["extract bot approach dist"] = ateam_geometry::norm(extract_robot.pos -
        approach_point_);
    if (ateam_geometry::norm(extract_robot.pos - approach_point_) < 0.03) {
      const bool use_pivot = true;
      if (use_pivot && !world.ball.visible) {
        getPlayInfo()["ExtractState"] = "pivoting";

        // This vector feels backwards but it works I guess
        const double direction = angles::shortest_angular_distance(
          ateam_geometry::ToHeading(extract_robot.pos - ateam_geometry::Point(0, 0)),
          extract_robot.theta
        );

        motion_command.twist.angular.z = std::copysign(0.8, direction);
      } else {
        state_ = State::Placing;
      }
    } else {
      MotionOptions motion_options;
      motion_options.completion_threshold = 0;
      emt.setMotionOptions(motion_options);
      path_planning::PlannerOptions planner_options = emt.getPlannerOptions();
      planner_options.avoid_ball = false;
      planner_options.footprint_inflation = -0.9 * kRobotRadius;
      emt.setPlannerOptions(planner_options);

      emt.setTargetPosition(approach_point_);
      emt.setMaxVelocity(0.2);
      emt.setMaxAccel(1.0);
      emt.setMaxDecel(1.0);
      if (world.ball.visible) {
        emt.face_point(world.ball.pos);

      // If the ball is occluded we sometimes drive past its previous position and try to turn
      // around so its better to just keep facing the same direction if we lose track of it
      } else {
        emt.face_absolute(extract_robot.theta);
      }

      motion_command = emt.runFrame(extract_robot, world);
      // motion_command.twist_frame = ateam_msgs::msg::RobotMotionCommand::FRAME_BODY;
      // motion_command.twist.linear.x = -0.1;
      // motion_command.twist.linear.y = 0.0;
    }
  } else if (robot_already_in_position || robot_near_approach_point) {
    getPlayInfo()["ExtractState"] = "capturing ball";
    MotionOptions motion_options;
    motion_options.completion_threshold = 0;
    emt.setMotionOptions(motion_options);
    path_planning::PlannerOptions planner_options = emt.getPlannerOptions();
    planner_options.avoid_ball = false;
    planner_options.footprint_inflation = -0.9 * kRobotRadius;
    emt.setPlannerOptions(planner_options);

    emt.setMaxVelocity(0.35);
    emt.setMaxAccel(2.0);
    emt.setMaxDecel(2.0);
    if (world.ball.visible) {
      emt.face_point(world.ball.pos);

    // If the ball is occluded we sometimes drive past its previous position and try to turn around
    // so its better to just keep facing the same direction if we lose track of it
    } else {
      emt.face_absolute(extract_robot.theta);
    }
    emt.setTargetPosition(world.ball.pos);
    motion_command = emt.runFrame(extract_robot, world);
    motion_command.twist_frame = ateam_msgs::msg::RobotMotionCommand::FRAME_BODY;
    motion_command.twist.linear.x = 0.35;
    motion_command.twist.linear.y = 0.0;
  } else {
    getPlayInfo()["ExtractState"] = "moving to approach point";
    emt.setMaxVelocity(1.5);
    emt.setMaxAccel(2.0);
    emt.setMaxDecel(2.0);
    emt.face_absolute(ateam_geometry::ToHeading(world.ball.pos - approach_point_));
    emt.setTargetPosition(approach_point_);
    motion_command = emt.runFrame(extract_robot, world);
  }

  const bool should_dribble = extract_robot.breakbeam_ball_detected_filtered ||
    ateam_geometry::norm(robot_to_ball) < 0.5;
  if (should_dribble) {
    motion_command.dribbler_speed = 300;
  }

  motion_commands[extract_robot.id] = motion_command;
}

void OurBallPlacementPlay::runPlacing(
  const std::vector<Robot> & available_robots, const World & world,
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
  16> & motion_commands)
{
  auto byDistToBall = [&world](const Robot & lhs, const Robot & rhs) {
      return CGAL::compare_distance_to_point(world.ball.pos, lhs.pos, rhs.pos) == CGAL::SMALLER;
    };

  const auto & place_robot = *std::min_element(
    available_robots.begin(),
    available_robots.end(), byDistToBall);

  getPlayInfo()["Assignments"]["Placer"] = place_robot.id;

  dribble_.setTarget(placement_point_);
  motion_commands[place_robot.id] = dribble_.runFrame(world, place_robot);
  ForwardPlayInfo(dribble_);
}

void OurBallPlacementPlay::runDone(
  const std::vector<Robot> & available_robots, const World & world,
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
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

  auto & emt = easy_move_tos_[place_robot.id];

  // TODO(chachmu): check next ref command to know if we need 0.5 for force start or
  // 0.05 for our free kick
  emt.setTargetPosition(world.ball.pos + (0.5 * ateam_geometry::normalize(ball_to_robot)));
  emt.face_point(world.ball.pos);
  auto command = emt.runFrame(place_robot, world);
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


}  // namespace ateam_kenobi::plays
