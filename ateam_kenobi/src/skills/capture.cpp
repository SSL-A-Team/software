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
#include "core/play_helpers/available_robots.hpp"

namespace ateam_kenobi::skills
{

Capture::Capture(stp::Options stp_options)
: stp::Skill(stp_options),
  easy_move_to_(createChild<play_helpers::EasyMoveTo>("EasyMoveTo"))
{}

void Capture::Reset()
{
  done_ = false;
  ball_detected_filter_ = 0;
  easy_move_to_.reset();
  fake_ball_point = ateam_geometry::Point(0, 0); // TESTING ONLY: REMOVE THIS
}

ateam_msgs::msg::RobotMotionCommand Capture::runFrame(const World & world, const Robot & robot)
{
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
    return approach_point;
  }

  // If ball moving: use intercept
  if (ateam_geometry::norm(world.ball.vel) > 0.15) {
    approach_point = calculateInterceptPoint(world, robot);
    return approach_point;
  }

  // TESTING ONLY: REMOVE THIS
  if (state_ == State::Intercept) {
    approach_point = calculateInterceptPoint(world, robot);
    return approach_point;
  }

  
  // Handle ball near goal/walls
  auto maybe_offset_vector = calculateObstacleOffset(world);
  if (maybe_offset_vector) {
    approach_point = world.ball.pos + maybe_offset_vector.value();
  }
  else  {
    approach_point = world.ball.pos + approach_radius_ * ateam_geometry::normalize(robot.pos - world.ball.pos);
  }

  return approach_point;
}


void Capture::chooseState(const World & world, const Robot & robot)
{

  calculateApproachPoint(world, robot);
  getOverlays().drawCircle("approach_point", ateam_geometry::makeCircle(approach_point, 0.025), "#0000FFFF");
  
  bool ballDetected = filteredBallSense(robot);
  getPlayInfo()["FilteredBallSense"] = ballDetected;

  bool ballTooCloseToObstacle = calculateObstacleOffset(world).has_value();
  
  const auto approach_to_ball = world.ball.pos - approach_point;
  const auto approach_to_ball_angle = std::atan2(approach_to_ball.y(), approach_to_ball.x());

  // Prioritize handling intercepting a moving ball
  // TODO: May need some hysteresis at the capture/intercept level to avoid vision filter issues
  if (ateam_geometry::norm(world.ball.vel) > 0.15) {
    state_ = State::Intercept;
  }

  // Transition out of Intercept
  else if (state_ == State::Intercept) {
    // state_ = State::MoveToApproachPoint;
    state_ = State::Intercept; // TESTING ONLY: REMOVE THIS
  }

  // TODO: Make the transition out of MoveToApproachPoint check if the robot
  // is already on the line from the approach point to the ball
  // so it doesn't try to go backwards to the approach point if it is already lined up

  // Transition out of MoveToApproachPoint
  else if (state_ == State::MoveToApproachPoint
    && ateam_geometry::norm(approach_point - robot.pos) < .04
    && abs(approach_to_ball_angle - robot.theta) < 0.1 ) {

    state_ = State::Capture;
  }

  // Transition out of Capture
  else if (state_ == State::Capture && ballDetected) {
    if (ballTooCloseToObstacle) {
      state_ = State::Extract;
    } else {
      done_ = true;
    }
  }

  // Transition out of Extract
  else if (state_ == State::Extract) {
    if (ballDetected) {
      // Ball is usually occluded by the robot so if we have possession and the robot has moved away
      // Then assume we successfully extracted
      if (!ballTooCloseToObstacle
          || ateam_geometry::norm(approach_point - robot.pos) < .02) {
        done_ = true;
      }

    // TODO: Could maybe have this check if the ball has moved into a position where we need to start over
    } else {
      state_ = State::Capture;
    }
  }
}

ateam_msgs::msg::RobotMotionCommand Capture::runIntercept(const World & world, const Robot & robot)
{

  easy_move_to_.face_point(world.ball.pos);
  MotionOptions motion_options;
  motion_options.completion_threshold = 0;
  easy_move_to_.setMotionOptions(motion_options);
  path_planning::PlannerOptions planner_options = easy_move_to_.getPlannerOptions();
  planner_options.avoid_ball = false;

  easy_move_to_.setPlannerOptions(planner_options);
  easy_move_to_.setMaxVelocity(2.0);
  easy_move_to_.setMaxDecel(6.0); // TODO: Figure out how to improve tracking without doing this
  easy_move_to_.setTargetPosition(approach_point);


  auto command = easy_move_to_.runFrame(robot, world);
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
  easy_move_to_.setMaxDecel(decel_limit_);
  //easy_move_to_.setTargetPosition(approach_point);
  easy_move_to_.setTargetPosition(robot.pos);

  getPlayInfo()["ApproachPoint"]["x"] = approach_point.x();
  getPlayInfo()["ApproachPoint"]["y"] = approach_point.y();
  return easy_move_to_.runFrame(robot, world);
}

ateam_msgs::msg::RobotMotionCommand Capture::runCapture(const World & world, const Robot & robot)
{

  /* TODO(chachmu): If we disable default obstacles do we need to check if the target is off the
   * field?
   */
  path_planning::PlannerOptions planner_options = easy_move_to_.getPlannerOptions();
  planner_options.avoid_ball = false;
  easy_move_to_.setPlannerOptions(planner_options);

  MotionOptions motion_options;
  motion_options.completion_threshold = 0;
  easy_move_to_.setMotionOptions(motion_options);

  easy_move_to_.setMaxVelocity(capture_speed_);
  easy_move_to_.face_point(world.ball.pos);

  easy_move_to_.setTargetPosition(world.ball.pos);
  auto command = easy_move_to_.runFrame(robot, world);

  // TESTING
  command.twist.linear.x = capture_speed_ * cos(robot.theta);
  command.twist.linear.y = capture_speed_ * sin(robot.theta);

  command.dribbler_speed = 300;

  return command;
}

ateam_msgs::msg::RobotMotionCommand Capture::runExtract(const World & world, const Robot & robot)
{

  // Face away from the direction of travel
  easy_move_to_.face_point(robot.pos + (robot.pos - approach_point));
  MotionOptions motion_options;
  motion_options.completion_threshold = 0;
  easy_move_to_.setMotionOptions(motion_options);
  path_planning::PlannerOptions planner_options = easy_move_to_.getPlannerOptions();
  planner_options.avoid_ball = false;

  easy_move_to_.setPlannerOptions(planner_options);
  easy_move_to_.setTargetPosition(approach_point);

  auto command = easy_move_to_.runFrame(robot, world);

  command.dribbler_speed = 300;

  return command;
}


ateam_geometry::Point Capture::calculateInterceptPoint(const World & world, const Robot & robot)
{

  // auto intercept_pos = world.ball.pos;
  auto fake_ball_vel = ateam_geometry::Vector(1.0, 0);
  fake_ball_point += fake_ball_vel / 100.0;
  getOverlays().drawCircle("Simulated Intercept Ball", ateam_geometry::makeCircle(fake_ball_point, 0.025), "#FF0000FF");

  // ball velocity
  // const double vb = ateam_geometry::norm(world.ball.vel);
  const double vb = ateam_geometry::norm(fake_ball_vel); // TESTING ONLY: REMOVE THIS
  // max robot velocity (slightly decreased to give robot time to prepare)
  const double vr = 0.8;

  // Need to get there a bit in front of the ball
  // const auto offset_ball_pos = world.ball.pos + ((kBallRadius + kRobotRadius + 0.05) * ateam_geometry::normalize(world.ball.vel));
  const auto offset_ball_pos = fake_ball_point + ((kBallRadius + kRobotRadius + 0.05) * ateam_geometry::normalize(fake_ball_vel)); // TESTING ONLY: REMOVE THIS
  //const auto ball_to_robot = robot.pos - world.ball.pos;
  const auto ball_to_robot = robot.pos - offset_ball_pos;

  // robot distance to ball projected onto the balls trajectory
  // const double d = (world.ball.vel * ball_to_robot) /
  //   ateam_geometry::norm(world.ball.vel);

  const double d = (fake_ball_vel * ball_to_robot) / // TESTING ONLY: REMOVE THIS
    ateam_geometry::norm(fake_ball_vel);

  // const auto robot_proj_ball = world.ball.vel * d /
  //   ateam_geometry::norm(world.ball.vel);

  const auto robot_proj_ball = fake_ball_vel * d / // TESTING ONLY: REMOVE THIS
    ateam_geometry::norm(fake_ball_vel);

  const auto robot_perp_ball = ball_to_robot - robot_proj_ball;

  // robot distance tangent to the balls trajectory
  const double h = ateam_geometry::norm(robot_perp_ball);

  
  getPlayInfo()["Ball"]["x"] = fake_ball_point.x();
  getPlayInfo()["Ball"]["y"] = fake_ball_point.y();
  getPlayInfo()["Ball"]["vel"] = vb;
  getPlayInfo()["Ball"]["vx"] = world.ball.vel.x();
  getPlayInfo()["Ball"]["vy"] = world.ball.vel.y();
  
  getPlayInfo()["Robot"]["x"] = robot.pos.x();
  getPlayInfo()["Robot"]["y"] = robot.pos.y();
  getPlayInfo()["Robot"]["vx"] = robot.vel.x();
  getPlayInfo()["Robot"]["vy"] = robot.vel.y();
  
  getPlayInfo()["Offset Target"]["x"] = offset_ball_pos.x();
  getPlayInfo()["Offset Target"]["y"] = offset_ball_pos.y();

  getPlayInfo()["Intercept"]["d"] = d;
  getPlayInfo()["Intercept"]["h"] = h;

  /*
    Initial Equation:
      d - vb*t = sin(acos(h/(vr*t))) * vr*t
    Solve for t:
      t = (d * vb - sqrt((d^2*vr^2) + (h^2 * vr^2) - (h^2 * vb^2)) / (vb^2 - vr^2)
  */

  double discriminant = (d * d * vr * vr) + (h * h * vr * vr) - (h * h * vb * vb);
  double denominator = vb * vb - vr * vr;

  // Imaginary result means we will never catch the ball
  if (discriminant < 0) {
    getPlayInfo()["Intercept"]["direction"] = ":(";
    getPlayInfo()["Intercept"]["Intercept Time"] = "inf";
    // Chase after the ball in case it hits something or slows down
    return world.ball.pos;
  }

  // If the robot and ball speeds are very similar then the denominator can go to 0
  if (abs(denominator) < 0.01) {
    // Choose a denominator representing a faster robot since:
    //  a. we already assume a slower than actual robot speed
    //  b. the ball will deccelerate as it rolls
    denominator = -0.1;
  }

  // Time to intercept moving towards the ball
  double t = (d * vb - sqrt(discriminant)) / denominator;

  // TODO(chachmu): add better checks to see if these times are possible to
  // respond to
  getPlayInfo()["Intercept"]["db_plus"] = t;
  if (t > 0.0) {
    getPlayInfo()["Intercept"]["Intercept Time"] = t;
    getPlayInfo()["Intercept"]["direction"] = "Toward";
    
    // return offset_ball_pos + (t * world.ball.vel);
    return offset_ball_pos + (t * fake_ball_vel);
  }

  // Time to intercept moving away from the ball
  t = (d * vb + sqrt(discriminant)) / denominator;
  
  getPlayInfo()["Intercept"]["db_minus"] = t;
  if (t > 0.0) {
    getPlayInfo()["Intercept"]["Intercept Time"] = t;
    getPlayInfo()["Intercept"]["direction"] = "Away";
    
    // return offset_ball_pos + (t * world.ball.vel);
    return offset_ball_pos + (t * fake_ball_vel);
  }

  getPlayInfo()["Intercept"]["Intercept Time"] = "inf";
  getPlayInfo()["Intercept"]["direction"] = ":(";

  // TODO: do something better here
  return robot.pos;
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
    bool is_near_back_goal_x = abs(world.ball.pos.x()) + ball_distance_from_obstacle > (world.field.field_length / 2) + world.field.goal_depth;
    bool is_near_inside_goalposts_y = abs(world.ball.pos.y()) + ball_distance_from_obstacle > world.field.goal_width / 2;

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
  bool is_near_outside_goalposts_x = abs(world.ball.pos.x()) + ball_distance_from_obstacle > world.field.field_length / 2;
  bool is_near_outside_goalposts_y = abs(world.ball.pos.y()) - ball_distance_from_obstacle < (world.field.goal_width / 2) + 0.02;
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
  bool is_near_x_walls =  abs(world.ball.pos.x()) + ball_distance_from_obstacle > (world.field.field_length / 2) + world.field.boundary_width;
  bool is_near_y_walls =  abs(world.ball.pos.y()) + ball_distance_from_obstacle > (world.field.field_width / 2) + world.field.boundary_width;
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

  // No obstacles to account for
  return std::nullopt;
}

bool Capture::filteredBallSense(const Robot & robot) {
  if (robot.breakbeam_ball_detected) {
    ball_detected_filter_ += 1;
    if (ball_detected_filter_ >= 20) {
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
