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
#include <vector>
#include <ateam_geometry/normalize.hpp>
#include "play_helpers/available_robots.hpp"

namespace ateam_kenobi::skills
{

Capture::Capture(stp::Options stp_options)
: stp::Skill(stp_options),
  easy_move_to_(createChild<play_helpers::EasyMoveTo>("EasyMoveTo"))
{}

void Capture::Reset()
{
  done_ = false;
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
  if (ateam_geometry::norm(world.ball.pos - robot.pos) < 0.2) {
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
  path_planning::PlannerOptions planner_options;
  planner_options.avoid_ball = false;
  planner_options.draw_obstacles = true;

  easy_move_to_.setPlannerOptions(planner_options);
  easy_move_to_.setTargetPosition(world.ball.pos);
  easy_move_to_.setMaxVelocity(1.5);

  return easy_move_to_.runFrame(robot, world);
}

ateam_msgs::msg::RobotMotionCommand Capture::runCapture(const World & world, const Robot & robot)
{
  // TODO: Should we filter this over a few frames to make sure we have the ball settled?
  if (robot.breakbeam_ball_detected) {
    done_ = true;
  }

  // TODO: If we disable default obstacles do we need to check if the target is off the field?
  path_planning::PlannerOptions planner_options;
  planner_options.avoid_ball = false;
  planner_options.draw_obstacles = true;
  easy_move_to_.setPlannerOptions(planner_options);

  easy_move_to_.setMaxVelocity(0.3);
  easy_move_to_.face_point(world.ball.pos);

  easy_move_to_.setTargetPosition(world.ball.pos);
  auto command = easy_move_to_.runFrame(robot, world);

  command.dribbler_speed = 200;

  return command;
}
}  // namespace ateam_kenobi::skills
