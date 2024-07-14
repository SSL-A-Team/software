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
#include <vector>
#include <ateam_geometry/normalize.hpp>
#include "play_helpers/available_robots.hpp"

namespace ateam_kenobi::skills
{

Dribble::Dribble(stp::Options stp_options)
: stp::Skill(stp_options),
  easy_move_to_(createChild<play_helpers::EasyMoveTo>("EasyMoveTo"))
{}

void Dribble::reset()
{
  done_ = false;
  easy_move_to_.reset();
}

ateam_msgs::msg::RobotMotionCommand Dribble::runFrame(const World & world, const Robot & robot)
{
  const auto start_position = getStartPosition(world);

  getOverlays().drawLine("dribble_line", {start_position, target_}, "#FFFF007F");

  chooseState(world, robot);

  switch (state_) {
    case State::MoveBehindBall:
      return runMoveBehindBall(world, robot);
    case State::Dribble:
      return runDribble(world, robot);
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
  switch (state_) {
    case State::MoveBehindBall:
      if (isRobotBehindBall(world, robot, 1.0) && isRobotSettled(world, robot)) {
        state_ = State::Dribble;
      }
      break;
    case State::Dribble:
      // Can eventually replace this with breakbeam
      if (!robotHasBall(world, robot)) {
        state_ = State::MoveBehindBall;
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

  const auto robot_vel_is_good = std::abs(robot_vel_perp_mag) < 0.2;
  return robot_vel_is_good;
}

bool Dribble::robotHasBall(const World & world, const Robot & robot)
{
  const auto robot_to_target = target_ - robot.pos;
  const auto robot_to_target_angle = std::atan2(robot_to_target.y(), robot_to_target.x());

  const auto robot_to_ball = world.ball.pos - robot.pos;

  const bool robot_touching_ball = ateam_geometry::norm(robot_to_ball) < kRobotRadius + 0.08;
  const bool mouth_facing_ball = std::abs(
    angles::shortest_angular_distance(
      robot.theta,
      robot_to_target_angle)) < 0.2;

  return robot_touching_ball && mouth_facing_ball;
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
  easy_move_to_.setPlannerOptions(planner_options);
  easy_move_to_.setTargetPosition(getStartPosition(world));
  easy_move_to_.setMaxVelocity(1.5);
  return easy_move_to_.runFrame(robot, world);
}

ateam_msgs::msg::RobotMotionCommand Dribble::runDribble(const World & world, const Robot & robot)
{
  /* TODO(chachmu): If we disable default obstacles do we need to check if the target is off the
   * field?
   */
  path_planning::PlannerOptions planner_options;
  planner_options.avoid_ball = false;
  planner_options.footprint_inflation = 0.0;
  planner_options.use_default_obstacles = false;
  planner_options.draw_obstacles = true;
  easy_move_to_.setPlannerOptions(planner_options);

  easy_move_to_.setMaxVelocity(0.3);
  easy_move_to_.face_point(target_);

  // Offset the robot position so the ball is on the target point
  const auto robot_to_target = target_ - robot.pos;
  easy_move_to_.setTargetPosition(
    target_ +
    (kRobotRadius * ateam_geometry::normalize(robot_to_target)));
  auto command = easy_move_to_.runFrame(robot, world);

  command.dribbler_speed = 50;

  return command;
}
}  // namespace ateam_kenobi::skills