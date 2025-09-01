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


#include "dribble.hpp"
#include <angles/angles.h>
#include <chrono>
#include <vector>
#include <ateam_geometry/normalize.hpp>
#include "core/play_helpers/available_robots.hpp"
#include <chrono>

namespace ateam_kenobi::skills
{

Dribble::Dribble(stp::Options stp_options)
: stp::Skill(stp_options),
  easy_move_to_(createChild<play_helpers::EasyMoveTo>("EasyMoveTo"))
{
  back_away_duration_ = std::chrono::seconds(1);
}

void Dribble::reset()
{
  done_ = false;
  ball_detected_filter_ = 0;
  easy_move_to_.reset();
}

ateam_msgs::msg::RobotMotionCommand Dribble::runFrame(const World & world, const Robot & robot)
{
  const auto start_position = getStartPosition(world);

  getOverlays().drawLine("dribble_line", {start_position, target_}, "#FFFF007F");

  chooseState(world, robot);

  switch (state_) {
    case State::MoveBehindBall:
      getPlayInfo()["State"] = "Move Behind Ball";
      return runMoveBehindBall(world, robot);
    case State::Dribble:
      getPlayInfo()["State"] = "Dribble to Point";
      return runDribble(world, robot);
    case State::BackAway:
      getPlayInfo()["State"] = "Back Away";
      return runBackAway(world, robot);

    default:
      std::cerr << "Unhandled state in dribble!\n";
      return ateam_msgs::msg::RobotMotionCommand{};
  }
}

ateam_geometry::Point Dribble::getStartPosition(const World & world)
{
  return world.ball.pos + (kOffset * ateam_geometry::normalize(
           world.ball.pos - target_));
}


void Dribble::chooseState(const World & world, const Robot & robot)
{
  const bool robot_has_ball = robotHasBall(robot);

  const double hysteresis = (state_ == State::BackAway) ? 1.0 : 0.5;

  const bool ball_in_zone = world.ball.visible &&
    ateam_geometry::norm(target_ - world.ball.pos) < hysteresis * target_threshold_;

  const auto robot_target_point = target_ -
    (kRobotRadius * ateam_geometry::normalize(target_ - robot.pos));
  const bool robot_has_ball_in_zone = robot_has_ball &&
    ateam_geometry::norm(robot_target_point - robot.pos) < hysteresis * target_threshold_;

  const auto dist_to_ball = ateam_geometry::norm(world.ball.pos - robot.pos);

  switch (state_) {
    case State::MoveBehindBall:
      if (robot_has_ball ||
        (isRobotBehindBall(world, robot, 1.0) && isRobotSettled(world, robot)))
      {
        state_ = State::Dribble;
      }
      break;
    case State::Dribble:
      if (ball_in_zone || robot_has_ball_in_zone) {
        state_ = State::BackAway;
        back_away_start_ = std::chrono::steady_clock::now();
      } else if (!robot_has_ball && !isRobotBehindBall(world, robot, 3.8)) {
        state_ = State::MoveBehindBall;
      }
      break;
    case State::BackAway:
      if (world.ball.visible && !ball_in_zone) {
        if (!robot_has_ball && !isRobotBehindBall(world, robot, 3.8)) {
          state_ = State::MoveBehindBall;
        }
      } else if (ball_in_zone && dist_to_ball > kRobotDiameter) {
        done_ = true;
      }
      break;
  }
}


bool Dribble::isRobotBehindBall(const World & world, const Robot & robot, double hysteresis)
{
  const auto ball_to_target = target_ - world.ball.pos;

  const auto robot_to_ball = world.ball.pos - robot.pos;

  const auto robot_proj_dist_to_ball = (ball_to_target * robot_to_ball) /
    ateam_geometry::norm(ball_to_target);

  const auto robot_proj_ball = ball_to_target * robot_proj_dist_to_ball /
    ateam_geometry::norm(ball_to_target);

  const auto robot_perp_ball = robot_to_ball - robot_proj_ball;
  const auto robot_perp_dist_to_ball = ateam_geometry::norm(robot_perp_ball);

  const auto proj_dist_is_good = robot_proj_dist_to_ball > 0.1 / hysteresis &&
    robot_proj_dist_to_ball < 0.22;
  const auto perp_dist_is_good = robot_perp_dist_to_ball < 0.007 * hysteresis;

  return proj_dist_is_good && perp_dist_is_good;
}

bool Dribble::isRobotSettled(const World & world, const Robot & robot)
{
  const auto ball_to_target = target_ - world.ball.pos;
  const auto robot_vel_proj_mag = (ball_to_target * robot.vel) /
    ateam_geometry::norm(ball_to_target);

  const auto robot_vel_proj = ball_to_target * robot_vel_proj_mag /
    ateam_geometry::norm(ball_to_target);

  const auto robot_vel_perp = robot.vel - robot_vel_proj;
  const auto robot_vel_perp_mag = ateam_geometry::norm(robot_vel_perp);

  const auto robot_vel_is_good = std::abs(robot_vel_perp_mag) < 0.35;
  return robot_vel_is_good;
}

bool Dribble::robotHasBall(const Robot & robot)
{
  getPlayInfo()["Filter Value"] = ball_detected_filter_;
  if (robot.breakbeam_ball_detected) {
    ball_detected_filter_ += 3;
    if (ball_detected_filter_ >= 25) {
      if (ball_detected_filter_ >= 50) {
        ball_detected_filter_ = 50;
      }
      getPlayInfo()["Robot Has Ball"] = "True";
      return true;
    }
  } else {
    ball_detected_filter_ -= 1;
    if (ball_detected_filter_ < 0) {
      ball_detected_filter_ = 0;
    }
  }

  getPlayInfo()["Robot Has Ball"] = "False";
  return false;
}

ateam_msgs::msg::RobotMotionCommand Dribble::runMoveBehindBall(
  const World & world,
  const Robot & robot)
{
  easy_move_to_.face_point(target_);

  MotionOptions motion_options;
  motion_options.completion_threshold = 0;
  easy_move_to_.setMotionOptions(motion_options);
  path_planning::PlannerOptions planner_options;
  planner_options.footprint_inflation = 0.06;
  planner_options.draw_obstacles = true;
  planner_options.ignore_start_obstacle = false;
  easy_move_to_.setPlannerOptions(planner_options);
  easy_move_to_.setTargetPosition(getStartPosition(world));
  easy_move_to_.setMaxVelocity(1.5);

  auto command = easy_move_to_.runFrame(robot, world);
  if (ateam_geometry::norm(robot.pos - world.ball.pos) < 0.5) {
    command.dribbler_speed = 130;
  }
  return command;
}

ateam_msgs::msg::RobotMotionCommand Dribble::runDribble(const World & world, const Robot & robot)
{
  path_planning::PlannerOptions planner_options;
  planner_options.avoid_ball = false;
  planner_options.footprint_inflation = 0.0;
  planner_options.use_default_obstacles = false;
  planner_options.draw_obstacles = true;
  easy_move_to_.setPlannerOptions(planner_options);

  easy_move_to_.setMaxVelocity(0.35);
  easy_move_to_.face_point(target_);

  // Offset the robot position so the ball is on the target point
  const auto robot_to_target = target_ - robot.pos;
  easy_move_to_.setTargetPosition(
    target_ -
    (kRobotRadius * ateam_geometry::normalize(robot_to_target)));
  auto command = easy_move_to_.runFrame(robot, world);

  command.dribbler_speed = 130;

  return command;
}

ateam_msgs::msg::RobotMotionCommand Dribble::runBackAway(const World & world, const Robot & robot)
{
  path_planning::PlannerOptions planner_options;
  planner_options.avoid_ball = false;
  planner_options.footprint_inflation = 0.0;
  planner_options.use_default_obstacles = false;
  planner_options.draw_obstacles = true;
  easy_move_to_.setPlannerOptions(planner_options);

  auto command = ateam_msgs::msg::RobotMotionCommand();
  command.dribbler_speed = 0;

  easy_move_to_.setMaxVelocity(0.35);
  easy_move_to_.face_absolute(robot.theta);

  const auto ball_to_robot = robot.pos - world.ball.pos;
  easy_move_to_.setTargetPosition(robot.pos +
    (0.2 * ateam_geometry::normalize(ball_to_robot)));

  // Wait for the dribbler to wind down before moving
  if ((std::chrono::steady_clock::now() - back_away_duration_.value()) > back_away_start_) {
    if (!world.ball.visible) {
      command.twist.linear.x = -0.35;
      command.twist_frame = ateam_msgs::msg::RobotMotionCommand::FRAME_BODY;
    } else {
      command = easy_move_to_.runFrame(robot, world);
    }
  }

  return command;
}
}  // namespace ateam_kenobi::skills
