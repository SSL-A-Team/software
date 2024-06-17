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


#include "line_kick.hpp"
#include <angles/angles.h>
#include <vector>
#include <ateam_geometry/normalize.hpp>

namespace ateam_kenobi::skills
{

LineKick::LineKick(stp::Options stp_options)
: stp::Skill(stp_options),
  easy_move_to_(createChild<play_helpers::EasyMoveTo>("EasyMoveTo"))
{
}

ateam_geometry::Point LineKick::getAssignmentPoint(const World & world)
{
  return getPreKickPosition(world);
}

ateam_msgs::msg::RobotMotionCommand LineKick::runFrame(const World & world, const Robot & robot)
{
  const auto pre_kick_position = getPreKickPosition(world);

  getOverlays().drawLine("kick_line", {pre_kick_position, target_point_}, "#FFFF007F");

  chooseState(world, robot);

  switch (state_) {
    case State::MoveBehindBall:
      return runMoveBehindBall(world, robot);
    case State::FaceBall:
      return runFaceBall(world, robot);
    case State::KickBall:
      return runKickBall(world, robot);
    case State::Done:
      return ateam_msgs::msg::RobotMotionCommand{};
    default:
      RCLCPP_WARN(getLogger(), "Unhandled state in line kick!");
      return ateam_msgs::msg::RobotMotionCommand{};
  }
}

ateam_geometry::Point LineKick::getPreKickPosition(const World & world)
{
  return world.ball.pos + (kPreKickOffset * ateam_geometry::normalize(
           world.ball.pos - target_point_));
}


void LineKick::chooseState(const World & world, const Robot & robot)
{
  switch (state_) {
    case State::MoveBehindBall:
      if (isRobotBehindBall(world, robot, 1.0) && isRobotSettled(world, robot)) {
        state_ = State::FaceBall;
      }
      break;
    case State::FaceBall:
      if (!isRobotBehindBall(world, robot, 3.0)) {
        state_ = State::MoveBehindBall;
      } else if (isRobotFacingBall(robot)) {
        state_ = State::KickBall;
      }
      break;
    case State::KickBall:
      if (!isRobotBehindBall(world, robot, 3.0)) {
        state_ = State::MoveBehindBall;
      } else if (isBallMoving(world)) {
        state_ = State::Done;
      }
      break;
    case State::Done:
      // Done is only exitted by resetting.
      break;
  }
}


bool LineKick::isRobotBehindBall(const World & world, const Robot & robot, double hysteresis)
{
  const auto ball_to_target = target_point_ - world.ball.pos;

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

bool LineKick::isRobotSettled(const World & world, const Robot & robot)
{
  const auto ball_to_target = target_point_ - world.ball.pos;
  const auto robot_vel_proj_mag = (ball_to_target * robot.vel) /
    ateam_geometry::norm(ball_to_target);

  const auto robot_vel_proj = ball_to_target * robot_vel_proj_mag /
    ateam_geometry::norm(ball_to_target);

  const auto robot_vel_perp = robot.vel - robot_vel_proj;
  const auto robot_vel_perp_mag = ateam_geometry::norm(robot_vel_perp);

  const auto robot_vel_is_good = std::abs(robot_vel_perp_mag) < 0.2;
  return robot_vel_is_good;
}

bool LineKick::isRobotFacingBall(const Robot & robot)
{
  const auto robot_to_target = target_point_ - robot.pos;
  const auto robot_to_target_angle = std::atan2(robot_to_target.y(), robot_to_target.x());
  return std::abs(
    angles::shortest_angular_distance(
      robot.theta,
      robot_to_target_angle)) < 0.05;
}

bool LineKick::isBallMoving(const World & world)
{
  return ateam_geometry::norm(world.ball.vel) > 0.5;
}

ateam_msgs::msg::RobotMotionCommand LineKick::runMoveBehindBall(
  const World & world,
  const Robot & robot)
{
  easy_move_to_.face_point(target_point_);

  MotionOptions motion_options;
  motion_options.completion_threshold = 0;
  easy_move_to_.setMotionOptions(motion_options);
  path_planning::PlannerOptions planner_options;
  planner_options.footprint_inflation = 0.04;
  planner_options.draw_obstacles = true;
  easy_move_to_.setPlannerOptions(planner_options);
  easy_move_to_.setTargetPosition(getPreKickPosition(world));
  return easy_move_to_.runFrame(robot, world);
}

ateam_msgs::msg::RobotMotionCommand LineKick::runFaceBall(const World & world, const Robot & robot)
{
  easy_move_to_.setTargetPosition(robot.pos);
  easy_move_to_.face_point(target_point_);
  return easy_move_to_.runFrame(robot, world);
}

ateam_msgs::msg::RobotMotionCommand LineKick::runKickBall(const World & world, const Robot & robot)
{
  path_planning::PlannerOptions planner_options;
  planner_options.avoid_ball = false;
  planner_options.footprint_inflation = 0.0;
  planner_options.use_default_obstacles = true;
  planner_options.draw_obstacles = true;
  easy_move_to_.setPlannerOptions(planner_options);

  // Handle robot angle
  // easy_move_to_.face_point(target_point_);
  easy_move_to_.face_point(world.ball.pos);
  easy_move_to_.setTargetPosition(world.ball.pos);
  auto command = easy_move_to_.runFrame(robot, world);

  // Override the velocity to move directly into the ball
  double velocity = 0.4;
  command.twist.linear.x = std::cos(robot.theta) * velocity;
  command.twist.linear.y = std::sin(robot.theta) * velocity;
  command.kick = ateam_msgs::msg::RobotMotionCommand::KICK_ON_TOUCH;
  command.kick_speed = kick_speed_;

  return command;
}
}  // namespace ateam_kenobi::skills
