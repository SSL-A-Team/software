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
}

ateam_msgs::msg::RobotMotionCommand Capture::runFrame(const World & world, const Robot & robot)
{
  chooseState(world, robot);

  switch (state_) {
    case State::MoveToBall:
      return runMoveToBall(world, robot);
    case State::Capture:
      return runCapture(world, robot);
    default:
      std::cerr << "Unhandled state in Capture!\n";
      return ateam_msgs::msg::RobotMotionCommand{};
  }
}

void Capture::chooseState(const World & world, const Robot & robot)
{
  if (ateam_geometry::norm(world.ball.pos - robot.pos) < approach_radius_) {
    state_ = State::Capture;
  } else {
    state_ = State::MoveToBall;
  }
}

ateam_msgs::msg::RobotMotionCommand Capture::runMoveToBall(
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
  easy_move_to_.setTargetPosition(world.ball.pos);

  const auto distance_to_ball = CGAL::approximate_sqrt(CGAL::squared_distance(robot.pos,
      world.ball.pos));

  const auto decel_distance = distance_to_ball - approach_radius_;

  const auto max_decel_vel = std::sqrt((2.0 * decel_limit_ * decel_distance) +
      (capture_speed_ * capture_speed_));

  easy_move_to_.setMaxVelocity(std::min(max_decel_vel, max_speed_));

  return easy_move_to_.runFrame(robot, world);
}

ateam_msgs::msg::RobotMotionCommand Capture::runCapture(const World & world, const Robot & robot)
{
  // TODO(chachmu): Should we filter this over a few frames to make sure we have the ball settled?
  if (robot.breakbeam_ball_detected) {
    ball_detected_filter_ += 1;
    if (ball_detected_filter_ >= 30) {
      done_ = true;
    }
  } else {
    ball_detected_filter_ -= 2;
    if (ball_detected_filter_ < 0) {
      ball_detected_filter_ = 0;
    }
  }

  /* TODO(chachmu): If we disable default obstacles do we need to check if the target is off the
   * field?
   */
  path_planning::PlannerOptions planner_options = easy_move_to_.getPlannerOptions();
  planner_options.avoid_ball = false;
  // planner_options.draw_obstacles = true;
  easy_move_to_.setPlannerOptions(planner_options);

  MotionOptions motion_options;
  motion_options.completion_threshold = 0;
  easy_move_to_.setMotionOptions(motion_options);

  easy_move_to_.setMaxVelocity(capture_speed_);
  easy_move_to_.face_point(world.ball.pos);

  easy_move_to_.setTargetPosition(world.ball.pos);
  auto command = easy_move_to_.runFrame(robot, world);

  command.dribbler_speed = 300;

  return command;
}
}  // namespace ateam_kenobi::skills
