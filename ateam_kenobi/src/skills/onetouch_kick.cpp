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


#include "onetouch_kick.hpp"
#include <angles/angles.h>
#include <vector>
#include <ateam_msgs/msg/robot_motion_command.hpp>
#include <ateam_geometry/normalize.hpp>

namespace ateam_kenobi::skills
{

OnetouchKick::OnetouchKick(stp::Options stp_options, KickSkill::WaitType wait_type)
: KickSkill(stp_options, wait_type),
  easy_move_to_(createChild<play_helpers::EasyMoveTo>("easy_move_to"))
{
}

ateam_geometry::Point OnetouchKick::GetAssignmentPoint(const World & world)
{
  return world.ball.pos;
}

ateam_msgs::msg::RobotMotionCommand OnetouchKick::RunFrame(const World & world, const Robot & robot)
{
  getPlayInfo()["onetouch robot id"] = robot.id;
  getPlayInfo()["ONETOUCH STATE"] = "";
  getPlayInfo()["mtn"] = easy_move_to_.motion_controller_.debug_json;

  if (done_) {
    return ateam_msgs::msg::RobotMotionCommand{};
  }



  // if ( prev_state_ == State::Wait && ateam_geometry::norm(world.ball.vel) < 0.1) {
  if (ateam_geometry::norm(world.ball.vel) < 0.1) {
    getOverlays().drawCircle("OnetouchKick_TEST_point", ateam_geometry::makeCircle(TEST_point_, 0.01), "#FF0000FF");
    prev_state_ = State::Wait;
    return Wait(world, robot);
  }

  TEST_point_ = ateam_geometry::Point(0,0);
  intercept_result_ = play_helpers::calculateIntercept(world, robot, 0.0);

  if (intercept_result_.intercept_pos.has_value()) {
    prep_point_ = intercept_result_.intercept_pos.value() + 
      (2 * kRobotRadius * ateam_geometry::normalize(world.ball.pos - target_point_));
    getOverlays().drawCircle("OnetouchKick_prep_point", ateam_geometry::makeCircle(prep_point_, 0.01), "#0000FFFF");
    getOverlays().drawLine("OnetouchKick_line", {intercept_result_.intercept_pos.value(), target_point_}, "#FFFF007F");
  } else {
    prev_state_ = State::Wait;
    return Wait(world, robot);
  }
  
  const bool prepared_position = (intercept_result_.h < 0.03) && (intercept_result_.d >= 1.5 * kRobotRadius);
  const bool prepared_angle = abs(angles::shortest_angular_distance(ateam_geometry::ToHeading(target_point_ - prep_point_), robot.theta)) < 0.1;
  const bool time_is_close = intercept_result_.t <= 0.2;

  if (prepared_position && prepared_angle && time_is_close) {
    prev_state_ = State::InterceptKick;
    return InterceptKick(world, robot);
  }
  prev_state_ = State::MoveToPrepPoint;
  return MoveToPrepPoint(world, robot);
}

ateam_msgs::msg::RobotMotionCommand OnetouchKick::Wait(
  const World & world,
  const Robot & robot)
{

  ateam_geometry::Vector TEST_vel = ateam_geometry::Vector(0.5, 0);
  TEST_point_ += 0.01 * TEST_vel;

  getPlayInfo()["ONETOUCH STATE"] = "TEST MOVING POINT";
  easy_move_to_.setTargetPosition(TEST_point_, TEST_vel);
  // easy_move_to_.setTargetPosition(TEST_point_);
  easy_move_to_.face_absolute(0.0);
  auto motion_command = easy_move_to_.runFrame(robot, world);


  // getPlayInfo()["ONETOUCH STATE"] = "WAIT";
  // easy_move_to_.setTargetPosition(robot.pos);
  // auto motion_command = easy_move_to_.runFrame(robot, world);
  return motion_command;
}

ateam_msgs::msg::RobotMotionCommand OnetouchKick::MoveToPrepPoint(const World & world, const Robot & robot)
{
  getPlayInfo()["ONETOUCH STATE"] = "MOVETOPREPPOINT";
  // const auto robot_to_target = target_point_ - robot.pos;
  // const auto robot_to_target_angle = std::atan2(robot_to_target.y(), robot_to_target.x());

  // const auto angle_error = angles::shortest_angular_distance(robot.theta, robot_to_target_angle);

  // ateam_msgs::msg::RobotMotionCommand command;
  // easy_move_to_.setTargetPosition(robot.pos);

  easy_move_to_.face_absolute(ateam_geometry::ToHeading(target_point_ - robot.pos));
  easy_move_to_.setTargetPosition(prep_point_, world.ball.vel);
  auto command = easy_move_to_.runFrame(robot, world);

  return command;
}

ateam_msgs::msg::RobotMotionCommand OnetouchKick::InterceptKick(const World & world, const Robot & robot)
{
  getPlayInfo()["ONETOUCH STATE"] = "INTERCEPTKICK";
  // easy_move_to_.setTargetPosition(robot.pos);
  easy_move_to_.face_absolute(ateam_geometry::ToHeading(target_point_ - prep_point_));

  path_planning::PlannerOptions planner_options;
  planner_options.avoid_ball = false;
  planner_options.footprint_inflation = 0.0;
  planner_options.use_default_obstacles = false;
  easy_move_to_.setPlannerOptions(planner_options);
  const auto target_point = intercept_result_.intercept_pos.value() + 
      (0.7 * kRobotRadius * ateam_geometry::normalize(world.ball.pos - target_point_));
  easy_move_to_.setTargetPosition(target_point, world.ball.vel);

  auto command = easy_move_to_.runFrame(robot, world);
  // command.dribbler_speed = 300;

  if (IsAllowedToKick()) {
    if(kick_type_ == KickType::Kick) {
      command.kick_request = ateam_msgs::msg::RobotMotionCommand::KR_KICK_TOUCH;
    } else {
      command.kick_request = ateam_msgs::msg::RobotMotionCommand::KR_CHIP_TOUCH;
    }
    command.kick_speed = 3.0; // TODO: CHANGE THIS BACK, TESTING ONLY
  }

  return command;
}

}  // namespace ateam_kenobi::skills
