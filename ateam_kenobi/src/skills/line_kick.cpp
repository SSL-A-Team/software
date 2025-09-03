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
#include <algorithm>
#include <vector>
#include <ateam_geometry/normalize.hpp>

namespace ateam_kenobi::skills
{

LineKick::LineKick(stp::Options stp_options, KickSkill::WaitType wait_type)
: KickSkill(stp_options, wait_type),
  easy_move_to_(createChild<play_helpers::EasyMoveTo>("EasyMoveTo"))
{
}

ateam_geometry::Point LineKick::GetAssignmentPoint(const World & world)
{
  return GetPreKickPosition(world);
}

RobotCommand LineKick::RunFrame(const World & world, const Robot & robot)
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
      getPlayInfo()["state"] = "Done";
      return RobotCommand{};
    default:
      RCLCPP_ERROR(getLogger(), "Unhandled state in line kick!");
      return RobotCommand{};
  }
}

ateam_geometry::Point LineKick::GetPreKickPosition(const World & world)
{
  return world.ball.pos + (pre_kick_offset * ateam_geometry::normalize(
           world.ball.pos - target_point_));
}


void LineKick::ChooseState(const World & world, const Robot & robot)
{
  getPlayInfo()["IsRobotBehindBall"] = IsRobotBehindBall(world, robot, 1.0);
  getPlayInfo()["IsRobotSettled"] = IsRobotSettled(world, robot);
  getPlayInfo()["IsRobotFacingBall"] = IsRobotFacingBall(robot);
  getPlayInfo()["IsAllowedToKick"] = IsAllowedToKick();
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

  const auto robot_vel_is_good = std::abs(robot_vel_perp_mag) < 0.4;
  return robot_vel_is_good;
}

bool LineKick::IsRobotFacingBall(const Robot & robot)
{
  const auto robot_to_target = target_point_ - robot.pos;
  const auto robot_to_target_angle = std::atan2(robot_to_target.y(), robot_to_target.x());
  getPlayInfo()["Target angle"] = robot_to_target_angle;
  getPlayInfo()["Robot angle"] = robot.theta;
  return std::abs(
    angles::shortest_angular_distance(
      robot.theta,
      robot_to_target_angle)) < angle_threshold;
}

bool LineKick::IsBallMoving(const World & world)
{
  return ateam_geometry::norm(world.ball.vel) > 0.1 * GetKickSpeed();
}

RobotCommand LineKick::RunMoveBehindBall(
  const World & world,
  const Robot & robot)
{
  RobotCommand command;

  const auto prekick_position = GetPreKickPosition(world);

  command.motion_intent.angular = motion::intents::angular::FacingIntent{target_point_};

  auto vel = ateam_geometry::Vector(0.002, 0);
  if (ateam_geometry::norm(world.ball.vel) > 0) {
    vel = world.ball.vel;
  }
  command.motion_intent.linear =
    motion::intents::linear::VelocityAtPositionIntent{prekick_position + (0.1 * world.ball.vel),
    vel};

  command.motion_intent.motion_options.completion_threshold = 0.0;
  command.motion_intent.planner_options.draw_obstacles = true;
  command.motion_intent.planner_options.footprint_inflation = std::min(0.015, pre_kick_offset);

  // TODO(barulicm): add accel limit support to motion intent
  // easy_move_to_.setMaxAccel(1.5);
  // easy_move_to_.setMaxDecel(1.5);

  double obstacle_radius_multiplier = 1.8;
  const auto robot_to_prekick = prekick_position - robot.pos;
  const auto ball_to_target = target_point_ - world.ball.pos;
  if (this->cowabunga && ateam_geometry::norm(robot_to_prekick) < 2.5 * kRobotDiameter) {
    command.motion_intent.planner_options.footprint_inflation = -0.1;
    obstacle_radius_multiplier = 5.0;
    getPlayInfo()["COWABUNGA MODE"] = "COWABUNGA";
  } else {
    getPlayInfo()["COWABUNGA MODE"] = "not cowabunga :(";
  }

  // Add additional obstacles to better avoid ball
  const auto angle = std::atan2(ball_to_target.y(), ball_to_target.x());
  command.motion_intent.obstacles = {
    // Front Obstacle
    ateam_geometry::makeDisk(
      world.ball.pos + kBallDiameter * ateam_geometry::Vector(
        std::cos(angle),
        std::sin(angle)),
      kBallRadius * obstacle_radius_multiplier),
    // Left Obstacle
    ateam_geometry::makeDisk(
      world.ball.pos + kBallDiameter * ateam_geometry::Vector(
        std::cos(angle + M_PI / 2),
        std::sin(angle + M_PI / 2)),
      kBallRadius * obstacle_radius_multiplier),
    // Right Obstacle
    ateam_geometry::makeDisk(
      world.ball.pos + kBallDiameter * ateam_geometry::Vector(
        std::cos(angle - M_PI / 2),
        std::sin(angle - M_PI / 2)),
      kBallRadius * obstacle_radius_multiplier)
  };

  return command;
}

RobotCommand LineKick::RunFaceBall(const World &, const Robot &)
{
  RobotCommand command;
  command.motion_intent.angular = motion::intents::angular::FacingIntent{target_point_};
  return command;
}

RobotCommand LineKick::RunKickBall(const World & world, const Robot &)
{
  RobotCommand command;

  command.motion_intent.linear =
    motion::intents::linear::VelocityIntent{ateam_geometry::Vector{kick_drive_velocity, 0.0},
    motion::intents::linear::Frame::Local};

  command.motion_intent.angular = motion::intents::angular::FacingIntent{world.ball.pos};

  if(KickOrChip() == KickSkill::KickChip::Kick) {
    command.kick = KickState::KickOnTouch;
  } else {
    command.kick = KickState::ChipOnTouch;
  }
  command.kick_speed = GetKickSpeed();

  return command;
}
}  // namespace ateam_kenobi::skills
