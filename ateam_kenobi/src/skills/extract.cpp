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

#include "extract.hpp"

namespace ateam_kenobi::skills
{

Extract::Extract(stp::Options stp_options)
: stp::Skill(stp_options),
  easy_move_to_(createChild<play_helpers::EasyMoveTo>("easy_move_to"))
{}

void Extract::Reset()
{
  easy_move_to_.reset();
  ballsense_count_ = 0;
  rip_start_time_.reset();
}

ateam_msgs::msg::RobotMotionCommand Extract::RunFrame(const World & world, const Robot & robot)
{
  if(robot.breakbeam_ball_detected) {
    ballsense_count_++;
  } else {
    ballsense_count_ = 0;
  }

  const auto time_since_start = rip_start_time_ ? (std::chrono::steady_clock::now() - *rip_start_time_) : std::chrono::seconds::max();
  const auto should_rip = ballsense_count_ > kBallsenseCountThreshold_ || time_since_start < std::chrono::milliseconds(500);

  const auto ball_far = ateam_geometry::norm(robot.pos - world.ball.pos) > 0.4;

  if(should_rip) {
    if(!rip_start_time_) {
      rip_start_time_ = std::chrono::steady_clock::now();
    }
    auto planner_options = easy_move_to_.getPlannerOptions();
    planner_options.avoid_ball = false;
    planner_options.footprint_inflation = -0.1;
    easy_move_to_.setPlannerOptions(planner_options);
    easy_move_to_.setTargetPosition(robot.pos + (ateam_geometry::normalize(robot.pos - world.ball.pos) * 0.25));
    easy_move_to_.setMaxAngularVelocity(2.0);
  } else if(ball_far) {
    auto planner_options = easy_move_to_.getPlannerOptions();
    planner_options.avoid_ball = false;
    planner_options.footprint_inflation = path_planning::PlannerOptions{}.footprint_inflation;
    easy_move_to_.setPlannerOptions(planner_options);
    easy_move_to_.setTargetPosition(world.ball.pos);
    easy_move_to_.setMaxVelocity(2.0);
  } else {
    auto planner_options = easy_move_to_.getPlannerOptions();
    planner_options.avoid_ball = false;
    planner_options.footprint_inflation = -0.1;
    easy_move_to_.setPlannerOptions(planner_options);
    easy_move_to_.setTargetPosition(world.ball.pos);
    easy_move_to_.setMaxVelocity(0.35);
  }

  easy_move_to_.face_point(world.ball.pos);
  auto command = easy_move_to_.runFrame(robot, world);
  if(robot.breakbeam_ball_detected) {
    command.dribbler_speed = 600;
  }
  return command;
}


}  // namespace ateam_kenobi::skills
