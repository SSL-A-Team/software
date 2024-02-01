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
    hysteresis = 3.0;
  }


  const auto ball_to_target = target_point_ - world.ball.pos;
  const auto ball_to_target_angle = std::atan2(ball_to_target.y(), ball_to_target.x());

  const auto robot_to_ball = world.ball.pos - robot.pos;

  // scalar projection (distance along the vector) of the robot to the ball 
  // along the ball to target vector. A negative value indicates we are on the wrong side of the ball
  const auto robot_proj_dist_to_ball = (ball_to_target * robot_to_ball)
    / ateam_geometry::norm(ball_to_target);

  // vector projection of the robot to the ball along the ball to target vector
  const auto robot_proj_ball = ball_to_target * robot_proj_dist_to_ball
    / ateam_geometry::norm(ball_to_target);

  const auto robot_perp_ball = robot_to_ball - robot_proj_ball;
  const auto robot_perp_dist_to_ball = ateam_geometry::norm(robot_perp_ball);

  // Check if: robot is behind the ball but not too far, robot is in line with the ball and target
  if (robot_proj_dist_to_ball < 0.12 || robot_proj_dist_to_ball > 0.22 || robot_perp_dist_to_ball > 0.025) {
    if (prev_state_ != State::MoveBehindBall) {
      easy_move_to_.reset();
      prev_state_ = State::MoveBehindBall;
    }

    easy_move_to_.face_absolute(ball_to_target_angle);

    /* We can enable this if we are too slow to get to the ball
    // Face travel until we get closer
    easy_move_to_.face_travel();
    if (ateam_geometry::norm(robot_to_ball) < 0.5) {
      easy_move_to_.face_absolute(ball_to_target_angle);
    }
    */

    MotionOptions motion_options;
    motion_options.completion_threshold = 0;
    easy_move_to_.setMotionOptions(motion_options);
    easy_move_to_.setPlannerOptions({});
    easy_move_to_.setTargetPosition(getPreKickPosition(world));
    std::vector<ateam_geometry::AnyShape> obstacles = {
      ateam_geometry::makeCircle(world.ball.pos, 0.02)
    };
    return easy_move_to_.runFrame(robot, world, obstacles);
  }

  const auto robot_to_target = target_point_ - robot.pos;
  const auto robot_to_target_angle = std::atan2(robot_to_target.y(), robot_to_target.x());
  if (abs(angles::shortest_angular_distance(robot.theta, robot_to_target_angle)) > 0.05 * hysteresis) {
    if (prev_state_ != State::FaceBall) {
      easy_move_to_.reset();
      prev_state_ = State::FaceBall;
    }

    easy_move_to_.setTargetPosition(robot.pos);
    easy_move_to_.face_point(target_point_);
    //easy_move_to_.setMaxAngularVelocity(1.0);
    return easy_move_to_.runFrame(robot, world);
  }

  if (prev_state_ != State::KickBall) {
    easy_move_to_.reset();
    prev_state_ = State::KickBall;
  }

  path_planning::PlannerOptions planner_options;
  planner_options.avoid_ball = false;
  planner_options.footprint_inflation = 0.0;
  planner_options.use_default_obstacles = true;
  planner_options.draw_obstacles = true;
  easy_move_to_.setPlannerOptions(planner_options);

  // Handle robot angle
  easy_move_to_.face_point(target_point_);
  easy_move_to_.setTargetPosition(world.ball.pos);
  auto command = easy_move_to_.runFrame(robot, world);

  // Override the velocity to move directly into the ball
  int velocity = 1.5;
  command.twist.linear.x = std::cos(robot.theta) * velocity;
  command.twist.linear.y = std::sin(robot.theta) * velocity;
  command.kick = ateam_msgs::msg::RobotMotionCommand::KICK_ON_TOUCH;
  command.kick_speed = kick_speed_;

  return command;
}

ateam_geometry::Point LineKick::getPreKickPosition(const World & world)
{
  return world.ball.pos + (kPreKickOffset * ateam_geometry::normalize(
           world.ball.pos - target_point_));
}
}  // namespace ateam_kenobi::skills
