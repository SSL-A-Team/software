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

LineKick::LineKick(stp::Options stp_options, KickSkill::WaitType wait_type)
: KickSkill(stp_options, wait_type),
  easy_move_to_(createChild<play_helpers::EasyMoveTo>("EasyMoveTo"))
{
  this->cowabunga = false;
}

ateam_geometry::Point LineKick::GetAssignmentPoint(const World & world)
{
  return GetPreKickPosition(world);
}

ateam_msgs::msg::RobotMotionCommand LineKick::RunFrame(const World & world, const Robot & robot)
{
  const auto pre_kick_position = GetPreKickPosition(world);

  getOverlays().drawLine("kick_line", {pre_kick_position, target_point_}, "#FFFF007F");

  ChooseState(world, robot);

  switch (state_) {
    case State::MoveBehindBall:
      getPlayInfo()["state"] = "Move Behind Ball";
      return RunMoveBehindBall(world, robot);
    case State::FaceBall:
      getPlayInfo()["state"] = "Face Ball";
      return RunFaceBall(world, robot);
    case State::KickBall:
      getPlayInfo()["state"] = "Kick Ball";
      return RunKickBall(world, robot);
    case State::Done:
      return ateam_msgs::msg::RobotMotionCommand{};
    default:
      RCLCPP_WARN(getLogger(), "Unhandled state in line kick!");
      return ateam_msgs::msg::RobotMotionCommand{};
  }
}

ateam_geometry::Point LineKick::GetPreKickPosition(const World & world)
{
  return world.ball.pos + (kPreKickOffset * ateam_geometry::normalize(
           world.ball.pos - target_point_));
}


void LineKick::ChooseState(const World & world, const Robot & robot)
{
  getPlayInfo()["IsRobotBehindBall"] = IsRobotBehindBall(world, robot, 1.0);
  getPlayInfo()["IsRobotSettled"] = IsRobotSettled(world, robot);
  getPlayInfo()["IsRobotFacingBall"] = IsRobotFacingBall(robot);
  switch (state_) {
    case State::MoveBehindBall:
      if (IsRobotBehindBall(world, robot, 1.0) && IsRobotSettled(world, robot)) {
        state_ = State::FaceBall;
      }
      break;
    case State::FaceBall:
      if (!IsRobotBehindBall(world, robot, 3.8)) {
        state_ = State::MoveBehindBall;
      } else if (IsRobotFacingBall(robot) && IsAllowedToKick()) {
        state_ = State::KickBall;
      }
      break;
    case State::KickBall:
      if (!IsAllowedToKick()) {
        state_ = State::FaceBall;
      } else if (!IsRobotBehindBall(world, robot, 3.8)) {
        state_ = State::MoveBehindBall;
      } else if (IsBallMoving(world)) {
        state_ = State::Done;
      }
      break;
    case State::Done:
      // Done is only exitted by resetting.
      break;
  }
}


bool LineKick::IsRobotBehindBall(const World & world, const Robot & robot, double hysteresis)
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
  const auto perp_dist_is_good = robot_perp_dist_to_ball <
    robot_perp_dist_to_ball_threshold * hysteresis;

  getPlayInfo()["IsRobotBehindBall_Perp"] = robot_perp_dist_to_ball;
  getPlayInfo()["IsRobotBehindBall_Proj"] = robot_proj_dist_to_ball;


  return proj_dist_is_good && perp_dist_is_good;
}

bool LineKick::IsRobotSettled(const World & world, const Robot & robot)
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

bool LineKick::IsRobotFacingBall(const Robot & robot)
{
  const auto robot_to_target = target_point_ - robot.pos;
  const auto robot_to_target_angle = std::atan2(robot_to_target.y(), robot_to_target.x());
  return std::abs(
    angles::shortest_angular_distance(
      robot.theta,
      robot_to_target_angle)) < angle_threshold;
}

bool LineKick::IsBallMoving(const World & world)
{
  return ateam_geometry::norm(world.ball.vel) > 0.1 * GetKickSpeed();
}

ateam_msgs::msg::RobotMotionCommand LineKick::RunMoveBehindBall(
  const World & world,
  const Robot & robot)
{
  easy_move_to_.face_point(target_point_);

  MotionOptions motion_options;
  motion_options.completion_threshold = 0;
  easy_move_to_.setMotionOptions(motion_options);
  path_planning::PlannerOptions planner_options = easy_move_to_.getPlannerOptions();
  planner_options.footprint_inflation = 0.04;

  // planner_options.draw_obstacles = true;

  // Add additional obstacles to better avoid ball
  const auto ball_to_target = target_point_ - world.ball.pos;
  const auto angle = std::atan2(ball_to_target.y(), ball_to_target.x());
  std::vector<ateam_geometry::AnyShape> obstacles;

  // Front Obstacle
  obstacles.push_back(
    ateam_geometry::makeDisk(
      world.ball.pos + kBallDiameter * ateam_geometry::Vector(
        std::cos(angle / 2),
        std::sin(angle / 2)),
      kBallRadius * 1.5
  ));

  // Left Obstacle
  obstacles.push_back(
    ateam_geometry::makeDisk(
      world.ball.pos + kBallDiameter * ateam_geometry::Vector(
        std::cos(angle + M_PI / 2),
        std::sin(angle + M_PI / 2)),
      kBallRadius * 1.5
  ));

  // Right Obstacle
  obstacles.push_back(
    ateam_geometry::makeDisk(
      world.ball.pos + kBallDiameter * ateam_geometry::Vector(
        std::cos(angle - M_PI / 2),
        std::sin(angle - M_PI / 2)),
      kBallRadius * 1.5
  ));

  easy_move_to_.setMaxVelocity(move_to_ball_velocity);
  easy_move_to_.setPlannerOptions(planner_options);
  easy_move_to_.setTargetPosition(GetPreKickPosition(world));

  auto command = easy_move_to_.runFrame(robot, world, obstacles);

  // Cowabunga it is
  if (this->cowabunga) {
    // We are stuck due to a path planner failure or something
    if (command.twist.linear.x == 0 && command.twist.linear.y == 0) {
      getPlayInfo()["Cowabunga Mode"] = "Active";


      // We are lined up enough to try to kick
      if (IsRobotBehindBall(world, robot, 3.5)) {
        getPlayInfo()["Cowabunga Mode State"] = "Kick the Ball";
        state_ = State::KickBall;
      }
      // We are at least sort of already lined up behind the ball
      // try to scoot around/along the obstacle
      else if (IsRobotBehindBall(world, robot, 5.0)) {  // NOLINT
        getPlayInfo()["Cowabunga Mode State"] = "Scoot Around";

        // planner_options.avoid_ball = false;
        // planner_options.footprint_inflation = -0.1;
        // planner_options.draw_obstacles = false;

        // easy_move_to_.setMaxVelocity(0.5);
        // easy_move_to_.setPlannerOptions(planner_options);

        // command = easy_move_to_.runFrame(robot, world);

        // Where we're going we don't need plans
        command.twist.linear.x = std::cos(robot.theta) * 0.5;
        command.twist.linear.y = std::sin(robot.theta) * 0.5;
      } else if (ateam_geometry::norm(robot.pos - world.ball.pos) < 1.0) {
        getPlayInfo()["Cowabunga Mode State"] = "Get Closer";

        planner_options.footprint_inflation = -0.1;
        easy_move_to_.setPlannerOptions(planner_options);
        easy_move_to_.setMaxVelocity(1.0);

        command = easy_move_to_.runFrame(robot, world);
      }
      getPlayInfo()["dist to ball"] = ateam_geometry::norm(robot.pos - world.ball.pos);
    }
  }

  return command;
}

ateam_msgs::msg::RobotMotionCommand LineKick::RunFaceBall(const World & world, const Robot & robot)
{
  easy_move_to_.setTargetPosition(robot.pos);
  easy_move_to_.face_point(target_point_);

  auto command = easy_move_to_.runFrame(robot, world);
  command.twist.linear.x = 0;
  command.twist.linear.y = 0;
  return command;
}

ateam_msgs::msg::RobotMotionCommand LineKick::RunKickBall(const World & world, const Robot & robot)
{
  path_planning::PlannerOptions planner_options = easy_move_to_.getPlannerOptions();
  planner_options.avoid_ball = false;
  planner_options.footprint_inflation = 0.0;
  easy_move_to_.setPlannerOptions(planner_options);

  // Handle robot angle
  // easy_move_to_.face_point(target_point_);
  easy_move_to_.face_point(world.ball.pos);
  easy_move_to_.setTargetPosition(world.ball.pos);
  auto command = easy_move_to_.runFrame(robot, world);

  // Override the velocity to move directly into the ball
  command.twist.linear.x = std::cos(robot.theta) * kick_drive_velocity;
  command.twist.linear.y = std::sin(robot.theta) * kick_drive_velocity;
  if(kick_type_ == KickType::Kick) {
    command.kick_request = ateam_msgs::msg::RobotMotionCommand::KR_KICK_TOUCH;
  } else {
    command.kick_request = ateam_msgs::msg::RobotMotionCommand::KR_CHIP_TOUCH;
  }
  command.kick_speed = GetKickSpeed();

  return command;
}
}  // namespace ateam_kenobi::skills
