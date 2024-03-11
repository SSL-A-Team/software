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

LineKick::LineKick(visualization::Overlays overlays)
: overlays_(overlays),
  easy_move_to_(overlays.getChild("EasyMoveTo"))
{
}

ateam_geometry::Point LineKick::getAssignmentPoint(const World & world)
{
  return getPreKickPosition(world);
}

ateam_msgs::msg::RobotMotionCommand LineKick::runFrame(const World & world, const Robot & robot)
{
  const auto pre_kick_position = getPreKickPosition(world);

  overlays_.drawLine("kick_line", {pre_kick_position, target_point_}, "#FFFF007F");

  float hysteresis = 1.0;
  // Make it harder to accidentally leave kick state
  if (prev_state_ == State::KickBall) {
    hysteresis = 2.0;
  }

  const auto distance_to_pre_kick = ateam_geometry::norm(robot.pos, pre_kick_position);
  if (distance_to_pre_kick > 0.05 * hysteresis) {
    if (prev_state_ != State::MoveToPreKick) {
      easy_move_to_.reset();
      prev_state_ = State::MoveToPreKick;
    }
    return moveToPreKick(world, robot);
  }

  const auto robot_to_ball = target_point_ - robot.pos;
  const auto robot_to_ball_angle = std::atan2(robot_to_ball.y(), robot_to_ball.x());
  if (angles::shortest_angular_distance(robot.theta, robot_to_ball_angle) > 0.05 * hysteresis) {
    if (prev_state_ != State::FaceBall) {
      easy_move_to_.reset();
      prev_state_ = State::FaceBall;
    }
    return faceBall(world, robot);
  }

  if (prev_state_ != State::KickBall) {
    easy_move_to_.reset();
    prev_state_ = State::KickBall;
  }

  return kickBall(world, robot);
}

ateam_geometry::Point LineKick::getPreKickPosition(const World & world)
{
  return world.ball.pos + (kPreKickOffset * ateam_geometry::normalize(
           world.ball.pos - target_point_));
}

ateam_msgs::msg::RobotMotionCommand LineKick::moveToPreKick(
  const World & world,
  const Robot & robot)
{
  easy_move_to_.setPlannerOptions({});
  easy_move_to_.setTargetPosition(getPreKickPosition(world));
  easy_move_to_.face_travel();
  std::vector<ateam_geometry::AnyShape> obstacles = {
    ateam_geometry::makeCircle(world.ball.pos, 0.02)
  };
  return easy_move_to_.runFrame(robot, world, obstacles);
}

ateam_msgs::msg::RobotMotionCommand LineKick::faceBall(const World & world, const Robot & robot)
{
  easy_move_to_.setPlannerOptions({});
  easy_move_to_.setTargetPosition(robot.pos);
  easy_move_to_.face_point(target_point_);
  easy_move_to_.setMaxAngularVelocity(1.0);
  return easy_move_to_.runFrame(robot, world);
}

ateam_msgs::msg::RobotMotionCommand LineKick::kickBall(const World & world, const Robot & robot)
{
  const auto robot_to_ball = (world.ball.pos - robot.pos);
  easy_move_to_.setTargetPosition(
    robot.pos +
    (0.1 * ateam_geometry::normalize(robot_to_ball)));
  easy_move_to_.face_point(target_point_);
  path_planning::PlannerOptions planner_options;
  planner_options.avoid_ball = false;
  planner_options.footprint_inflation = 0.0;
  planner_options.use_default_obstacles = false;
  easy_move_to_.setPlannerOptions(planner_options);
  auto command = easy_move_to_.runFrame(robot, world);
  command.twist.linear.x = std::cos(robot.theta) * 0.5;
  command.twist.linear.y = std::sin(robot.theta) * 0.5;
  command.kick = ateam_msgs::msg::RobotMotionCommand::KICK_ON_TOUCH;
  command.kick_speed = 5.0;
  return command;
}
}  // namespace ateam_kenobi::skills
