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


#include "capture.hpp"
#include <angles/angles.h>
#include <algorithm>
#include <vector>
#include <ateam_geometry/normalize.hpp>
#include <ateam_common/angle.hpp>
#include "core/play_helpers/available_robots.hpp"

namespace ateam_kenobi::skills
{

Capture::Capture(stp::Options stp_options)
: stp::Skill(stp_options),
  easy_move_to_(createChild<play_helpers::EasyMoveTo>("EasyMoveTo"))
{}

void Capture::Reset()
{
  state_ = State::MoveToApproachPoint;
  done_ = false;
  ball_detected_filter_ = 0;
  easy_move_to_.reset();
}

ateam_msgs::msg::RobotMotionCommand Capture::runFrame(const World & world, const Robot & robot)
{
  getPlayInfo() = nlohmann::json();
  chooseState(world, robot);

  switch (state_) {
    case State::Intercept:
      getPlayInfo()["State"] = "Intercept";
      return runIntercept(world, robot);
    case State::MoveToApproachPoint:
      getPlayInfo()["State"] = "Move To Approach Point";
      return runMoveToApproachPoint(world, robot);
    case State::Capture:
      getPlayInfo()["State"] = "Capture";
      return runCapture(world, robot);
    case State::Extract:
      getPlayInfo()["State"] = "Extract";
      return runExtract(world, robot);
    default:
      std::cerr << "Unhandled state in Capture!\n";
      return ateam_msgs::msg::RobotMotionCommand{};
  }
}

ateam_geometry::Point Capture::calculateApproachPoint(const World & world, const Robot & robot)
{

  // Latch the approach point while in extract
  if (state_ == State::Extract) {
    return approach_point_;
  }

  // If ball moving: use intercept
  // Require a higher ball vel if capturing so we don't accidentally trigger intercept
  const double ball_vel = ateam_geometry::norm(world.ball.vel);
  if ((ball_vel > 0.15 && state_ != State::Capture) || (ball_vel > capture_speed_)) {
    approach_point_ = calculateInterceptPoint(world, robot);
    return approach_point_;
  }

  // Handle ball near goal/walls/opponents
  auto maybe_offset_vector = calculateObstacleOffset(world);
  if (maybe_offset_vector) {
    approach_point_ = world.ball.pos + maybe_offset_vector.value();
  }
  else  {
    approach_point_ = world.ball.pos + approach_radius_ * ateam_geometry::normalize(robot.pos - world.ball.pos);
  }

  return approach_point_;
}


void Capture::chooseState(const World & world, const Robot & robot)
{

  getPlayInfo()["mtn"] = easy_move_to_.motion_controller_.debug_json;

  calculateApproachPoint(world, robot);
  getOverlays().drawCircle("approach_point", ateam_geometry::makeCircle(approach_point_, 0.025), "#0000FFFF");
  getOverlays().drawLine("ballvel_line", {world.ball.pos, world.ball.pos + (0.3 * world.ball.vel)}, "#FF00007F");

  getPlayInfo()["ApproachPoint"]["x"] = approach_point_.x();
  getPlayInfo()["ApproachPoint"]["y"] = approach_point_.y();

  const bool ballDetected = filteredBallSense(robot);
  getPlayInfo()["FilteredBallSense"] = ballDetected;

  const bool ballTooCloseToObstacle = calculateObstacleOffset(world).has_value();
  getPlayInfo()["ballTooCloseToObstacle"] = ballTooCloseToObstacle;

  getPlayInfo()["ball_velocity"] = ateam_geometry::norm(world.ball.vel);
  getPlayInfo()["robot_velocity"] = ateam_geometry::norm(robot.vel);
  
  const auto robot_to_ball = world.ball.pos - robot.pos;
  const double robot_to_ball_angle = std::atan2(robot_to_ball.y(), robot_to_ball.x());

  const double ball_distance = ateam_geometry::norm(robot_to_ball);
  const double ball_vel = ateam_geometry::norm(world.ball.vel);

  // TODO: REMOVE THIS TESTING ONLY
    const auto ball_to_approach = approach_point_ - world.ball.pos;
    // const double ball_to_approach_angle = std::atan2(ball_to_approach.y(), ball_to_approach.x());
    const double robot_approach_angle_offset = std::acos((-robot_to_ball * ball_to_approach)
      / (ateam_geometry::norm(robot_to_ball) * ateam_geometry::norm(ball_to_approach))
    ); 
    getPlayInfo()["robot_approach_offset"] = robot_approach_angle_offset;
    // TODO: the above angle is super janky???

  // TODO: May need some hysteresis at the capture/intercept level to avoid vision filter issues

  // Robot has the ball
  if (ballDetected) {
    if (world.ball.visible) {
      if (ballTooCloseToObstacle) {
        getPlayInfo()["reason"] = "setting extract: ball detected close to obstacle";
        state_ = State::Extract;
      } else {
        getPlayInfo()["DONE"] = "CAPTURE HAS COMPLETED";
        done_ = true;
      }
    }
    // Can't see the ball but needed to be extracted
    else if (ballTooCloseToObstacle) {
      getPlayInfo()["reason"] = "setting extract: ball detected no vision";
      state_ = State::Extract;
    }
    // Vision is obstructed but we have moved far enough away
    else if (ateam_geometry::norm(approach_point_ - robot.pos) < .02) {
        getPlayInfo()["DONE"] = "CAPTURE HAS COMPLETED";
        done_ = true;
    }
  }

  // TODO: May need some hysteresis at the capture/intercept level to avoid vision filter issues
  // Require a higher ball vel if capturing so we don't accidentally trigger intercept
  else if ((ball_vel > 0.15 && state_ != State::Capture) || (ball_vel > capture_speed_)) {
    getPlayInfo()["reason"] = "setting intercept: bv>0.15";
    state_ = State::Intercept;
  }

  // Robot needs to go to the ball
  else {
    const bool robot_facing_ball = abs(robot_to_ball_angle - robot.theta) < 0.1;

    // Transition out of MoveToApproachPoint
    if (state_ == State::MoveToApproachPoint) {
      // Close to the approach point and facing the ball
      if (ateam_geometry::norm(approach_point_ - robot.pos) < .04
        && robot_facing_ball ) {
        getPlayInfo()["reason"] = "setting capture: at approach";
        state_ = State::Capture;
      }
    }

    // Transition into starting state
    else if (state_ != State::Capture) {
      // const auto ball_to_approach = approach_point - world.ball.pos;
      // const double ball_to_approach_angle = std::atan2(ball_to_approach.y(), ball_to_approach.x());
      // const double robot_approach_angle_offset = std::acos((-robot_to_ball * ball_to_approach)
      //   / (ateam_geometry::norm(robot_to_ball) * ateam_geometry::norm(ball_to_approach))       
      // ); 


      const bool robot_near_ball = ball_distance <= approach_radius_;
      const bool robot_already_in_position = robot_near_ball && robot_facing_ball
        && abs(robot_approach_angle_offset) < 0.3;

      const bool can_bypass_approach = !ballTooCloseToObstacle && (!always_approach_first_ || robot_already_in_position);


      // the ball is clear of obstacles, just use capture directly
      if (can_bypass_approach) {
        getPlayInfo()["reason"] = "setting capture: already in position";
        state_ = State::Capture;
      
      // Otherwise we have to go to the approach point first
      } else {
        state_ = State::MoveToApproachPoint;
      }
    }

    // Only need to handle the ball moving away since capture also ends above when breakbeam detects the ball
    else if (ball_distance > 2 * approach_radius_ || (abs(robot_to_ball_angle - robot.theta) > 0.2)){
      state_ = State::MoveToApproachPoint;
    }
  }
}

ateam_msgs::msg::RobotMotionCommand Capture::runIntercept(const World & world, const Robot & robot)
{

  MotionOptions motion_options;
  motion_options.completion_threshold = 0;
  easy_move_to_.setMotionOptions(motion_options);
  path_planning::PlannerOptions planner_options = easy_move_to_.getPlannerOptions();
  planner_options.footprint_inflation = 0.06;
  planner_options.avoid_ball = true;
  planner_options.force_replan = true;
  planner_options.draw_obstacles = true;

  // This angle seems likes its flipped 180?
  // double angle = atan2(world.ball.vel.y(), world.ball.vel.x());
  std::vector<ateam_geometry::AnyShape> obstacles;


  const auto target_point_to_ball = world.ball.pos - approach_point_;
  const auto robot_to_ball = world.ball.pos - robot.pos;

  double target_angle = ateam_common::geometry::VectorToAngle(target_point_to_ball);

  getPlayInfo()["intercept angle mode"] = "";
  // Aligned in front of the ball
  if (abs(ateam_geometry::ShortestAngleBetween(target_point_to_ball, robot_to_ball)) < M_PI / 8.0) {
    getPlayInfo()["intercept angle mode"] = "aligned";
    target_angle = ateam_common::geometry::VectorToAngle(world.ball.pos - robot.pos);
    planner_options.avoid_ball = false;
  } else {
    // Still directly behind the ball, not sure which way to face yet
    if (abs(ateam_geometry::ShortestAngleBetween(target_point_to_ball, robot_to_ball)) > 7 * M_PI / 8.0) {
      getPlayInfo()["intercept angle mode"] = "dont know";
      // target_angle = robot.theta;
      target_angle = ateam_common::geometry::VectorToAngle(world.ball.pos - robot.pos);
    } else {
      getPlayInfo()["intercept angle mode"] = "looping around";
    }
  }

  easy_move_to_.setPlannerOptions(planner_options);
  easy_move_to_.setMaxVelocity(2.5);
  easy_move_to_.setMaxAngularVelocity(4.0);
  easy_move_to_.setMaxAccel(6.0); // TODO: Figure out how to improve tracking without doing this
  easy_move_to_.setMaxDecel(6.0); // TODO: Figure out how to improve tracking without doing this

  easy_move_to_.face_absolute(target_angle);
  easy_move_to_.setTargetPosition(approach_point_, intercept_velocity_);
  getPlayInfo()["Intercept"]["intercept velocity"]["x"] = intercept_velocity_.x();
  getPlayInfo()["Intercept"]["intercept velocity"]["y"] = intercept_velocity_.y();

  auto command = easy_move_to_.runFrame(robot, world, obstacles);

  const double offset_ball_dist = kBallRadius + kRobotRadius + 0.05; // TODO be smarter about where to calculate this
  if (intercept_result_.h < 0.03 && intercept_result_.d >= -offset_ball_dist) {
    command.dribbler_speed = 300;
  }

  return command;
}

ateam_msgs::msg::RobotMotionCommand Capture::runMoveToApproachPoint(
  const World & world,
  const Robot & robot)
{
  easy_move_to_.face_point(world.ball.pos);

  MotionOptions motion_options;
  motion_options.completion_threshold = 0;
  easy_move_to_.setMotionOptions(motion_options);
  path_planning::PlannerOptions planner_options = easy_move_to_.getPlannerOptions();
  planner_options.avoid_ball = false;

  easy_move_to_.setPlannerOptions(planner_options);
  easy_move_to_.setMaxVelocity(2.0);
  easy_move_to_.setMaxAngularVelocity(2.0);
  easy_move_to_.setMaxAccel(3.0);
  easy_move_to_.setMaxDecel(3.0);
  easy_move_to_.setTargetPosition(approach_point_);

  return easy_move_to_.runFrame(robot, world);
}

ateam_msgs::msg::RobotMotionCommand Capture::runCapture(const World & world, const Robot & robot)
{

  /* TODO(chachmu): If we disable default obstacles do we need to check if the target is off the
   * field?
   */
  path_planning::PlannerOptions planner_options = easy_move_to_.getPlannerOptions();
  planner_options.avoid_ball = false;
  planner_options.footprint_inflation = -0.07;
  easy_move_to_.setPlannerOptions(planner_options);

  MotionOptions motion_options;
  motion_options.completion_threshold = 0;
  easy_move_to_.setMotionOptions(motion_options);

  easy_move_to_.setMaxVelocity(capture_speed_); // depends on robot tuning but I don't think we need to drop this speed lower than normal
  easy_move_to_.setMaxAngularVelocity(2.0);
  easy_move_to_.setMaxAccel(3.0);
  easy_move_to_.setMaxDecel(capture_decel_limit_); // TODO: Figure out how to improve tracking without doing this

  if (world.ball.visible) {
    easy_move_to_.face_point(world.ball.pos);

  // If the ball is occluded we sometimes drive past its previous position and try to turn around
  // so its better to just keep facing the same direction if we lose track of it
  } else {
    easy_move_to_.face_absolute(robot.theta);
  }

  // Put the dribbler on the ball, not the center of the robot
  const auto target_pos = world.ball.pos + 0.6 * kRobotRadius * (robot.pos - world.ball.pos);
  easy_move_to_.setTargetPosition(target_pos);
  auto command = easy_move_to_.runFrame(robot, world);

  command.dribbler_speed = 300;

  return command;
}

ateam_msgs::msg::RobotMotionCommand Capture::runExtract(const World & world, const Robot & robot)
{

  // Maintain our facing direction
  easy_move_to_.face_absolute(robot.theta);
  MotionOptions motion_options;
  motion_options.completion_threshold = 0;
  easy_move_to_.setMotionOptions(motion_options);
  path_planning::PlannerOptions planner_options = easy_move_to_.getPlannerOptions();
  planner_options.avoid_ball = false;

  easy_move_to_.setPlannerOptions(planner_options);
  easy_move_to_.setTargetPosition(approach_point_);
  easy_move_to_.setMaxVelocity(capture_speed_);
  easy_move_to_.setMaxAngularVelocity(2.0);
  easy_move_to_.setMaxAccel(1.0);
  easy_move_to_.setMaxDecel(3.0);

  auto command = easy_move_to_.runFrame(robot, world);

  command.dribbler_speed = 300;

  return command;
}


ateam_geometry::Point Capture::calculateInterceptPoint(const World & world, const Robot & robot)
{

  const double vb = ateam_geometry::norm(world.ball.vel);

  const double offset_ball_dist = kBallRadius + kRobotRadius + 0.25;

  intercept_result_ = play_helpers::calculateIntercept(world, robot, offset_ball_dist);
  
  getPlayInfo()["Ball"]["x"] = world.ball.pos.x();
  getPlayInfo()["Ball"]["y"] = world.ball.pos.y();
  getPlayInfo()["Ball"]["vel"] = vb;
  getPlayInfo()["Ball"]["vx"] = world.ball.vel.x();
  getPlayInfo()["Ball"]["vy"] = world.ball.vel.y();
  
  getPlayInfo()["Robot"]["x"] = robot.pos.x();
  getPlayInfo()["Robot"]["y"] = robot.pos.y();
  getPlayInfo()["Robot"]["vx"] = robot.vel.x();
  getPlayInfo()["Robot"]["vy"] = robot.vel.y();
  
  getPlayInfo()["Intercept"]["d"] = intercept_result_.d;
  getPlayInfo()["Intercept"]["h"] = intercept_result_.h;

  const bool facing_ball = abs(angles::shortest_angular_distance(ateam_geometry::ToHeading(world.ball.pos - robot.pos), robot.theta)) < 0.1;
  const bool aligned_perpindicularly = intercept_result_.h < 0.03;

  intercept_velocity_ = world.ball.vel;

  getPlayInfo()["debug"]["aligned_perp"] = aligned_perpindicularly;
  getPlayInfo()["debug"]["facing"] = facing_ball;
  getPlayInfo()["debug"]["offset"] = intercept_result_.d >= -offset_ball_dist;

  // Already aligned to receive the ball and are in front of it, capture it
  if (aligned_perpindicularly && facing_ball && intercept_result_.d >= -offset_ball_dist) {
    // Go to the ball
    if (vb < 0.01 || intercept_result_.d >= 10) {
    // if (vb < 0.5 || intercept_result_.d >= 0.3) { // PUT THIS BACK
      getPlayInfo()["Intercept"]["direction"] = "GO TO CAPTURE";
      return world.ball.pos;
    }
    // Wait for the ball to come to us
    else {
      getPlayInfo()["Intercept"]["direction"] = "WAIT FOR CAPTURE";

      intercept_velocity_ = world.ball.vel + 0.2 * ateam_geometry::normalize(world.ball.pos - robot.pos);
      return robot.pos + intercept_result_.robot_perp_ball;
    }
  }
  
  if (!intercept_result_.intercept_pos.has_value()) {
    getPlayInfo()["Intercept"]["direction"] = ":(";
    getPlayInfo()["Intercept"]["Intercept Time"] = "inf";
    // Chase after the ball in case it hits something or slows down
    return world.ball.pos + offset_ball_dist * ateam_geometry::normalize(world.ball.vel);
  }

  getPlayInfo()["Intercept"]["Intercept Time"] = intercept_result_.t;

  if (intercept_result_.equation_sign < 0) {
    getPlayInfo()["Intercept"]["direction"] = "Toward";
  } else {
    getPlayInfo()["Intercept"]["direction"] = "Away";
  }
  
  // Go to the intercept point
  return intercept_result_.intercept_pos.value();
}

std::optional<ateam_geometry::Vector> Capture::calculateObstacleOffset(const World & world)
{
  double obstacle_offset_x = 0;
  double obstacle_offset_y = 0;

  // Handle ball inside goal
  bool is_inside_goal_x = abs(world.ball.pos.x()) > world.field.field_length / 2;
  bool is_between_goalposts = abs(world.ball.pos.y()) < world.field.goal_width / 2;
  bool is_inside_goal = is_inside_goal_x && is_between_goalposts;

  if (is_inside_goal) {
    bool is_near_back_goal_x = abs(world.ball.pos.x()) + ball_distance_from_obstacle_ > (world.field.field_length / 2) + world.field.goal_depth;
    bool is_near_inside_goalposts_y = abs(world.ball.pos.y()) + ball_distance_from_obstacle_ > world.field.goal_width / 2;

    if (is_near_back_goal_x) {
      obstacle_offset_x = (world.ball.pos.x() > 0) ? -1.0 : 1.0;
    }
    if (is_near_inside_goalposts_y) {
      obstacle_offset_y = (world.ball.pos.y() > 0) ? -1.0 : 1.0;
    }

    auto offset_vector = approach_radius_ * ateam_geometry::normalize(ateam_geometry::Vector(obstacle_offset_x, obstacle_offset_y));
    return std::make_optional(offset_vector);
  }

  // Handle ball near outside goalposts
  bool is_near_outside_goalposts_x = abs(world.ball.pos.x()) + ball_distance_from_obstacle_ > world.field.field_length / 2;
  bool is_near_outside_goalposts_y = abs(world.ball.pos.y()) - ball_distance_from_obstacle_ < (world.field.goal_width / 2) + 0.02;
  bool is_near_outside_goalposts = is_near_outside_goalposts_x && is_near_outside_goalposts_y;

  if (is_near_outside_goalposts) {
    if (is_near_outside_goalposts_x) {
      obstacle_offset_x = (world.ball.pos.x() > 0) ? -1.0 : 1.0;
    }
    if (is_near_outside_goalposts_y) {
      obstacle_offset_y = (world.ball.pos.y() > 0) ? 1.0 : -1.0;
    }

    auto offset_vector = approach_radius_ * ateam_geometry::normalize(ateam_geometry::Vector(obstacle_offset_x, obstacle_offset_y));
    return std::make_optional(offset_vector);
  }

  // Handle ball near walls
  bool is_near_x_walls =  abs(world.ball.pos.x()) + ball_distance_from_obstacle_ > (world.field.field_length / 2) + world.field.boundary_width;
  bool is_near_y_walls =  abs(world.ball.pos.y()) + ball_distance_from_obstacle_ > (world.field.field_width / 2) + world.field.boundary_width;
  bool is_near_wall =  is_near_x_walls || is_near_y_walls;

  if (is_near_wall) {
    if (is_near_x_walls) {
      obstacle_offset_x = (world.ball.pos.x() > 0) ? -1.0 : 1.0;
    }
    if (is_near_y_walls) {
      obstacle_offset_y = (world.ball.pos.y() > 0) ? -1.0 : 1.0;
    }

    auto offset_vector = approach_radius_ * ateam_geometry::normalize(ateam_geometry::Vector(obstacle_offset_x, obstacle_offset_y));
    return std::make_optional(offset_vector);
  }

  // TODO: handle ball being held by opponent?
  for (const auto & their_robot : world.their_robots) {

    // Check if the ball is close enough to an opponent robot to require an offset
    if (their_robot.visible && ateam_geometry::norm(their_robot.pos - world.ball.pos) < ball_distance_from_obstacle_) {
      auto offset_vector = approach_radius_ * ateam_geometry::normalize(world.ball.pos - their_robot.pos);
      getPlayInfo()["Intercept"]["OPPONENT_COLLISION"] = their_robot.id;
      return std::make_optional(offset_vector);
    }
  }

  // No obstacles to account for
  return std::nullopt;
}

bool Capture::filteredBallSense(const Robot & robot) {
  if (robot.breakbeam_ball_detected) {
    ball_detected_filter_ += 1;
    if (ball_detected_filter_ > 80) {
      ball_detected_filter_ = 80;
    }
    if (ball_detected_filter_ >= 30) {
      return true;
    }
  } else {
    ball_detected_filter_ -= 2;
    if (ball_detected_filter_ < 0) {
      ball_detected_filter_ = 0;
    }
  }

  return false;
}

}  // namespace ateam_kenobi::skills
