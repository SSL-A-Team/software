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

#include "pass_receiver.hpp"
#include "play_helpers/available_robots.hpp"
#include <ateam_common/robot_constants.hpp>

namespace ateam_kenobi::skills
{

PassReceiver::PassReceiver(stp::Options stp_options)
: stp::Skill(stp_options),
  easy_move_to_(createChild<play_helpers::EasyMoveTo>("EasyMoveTo")),
  capture_(createChild<skills::Capture>("capture"))
{}

void PassReceiver::reset()
{
  done_ = false;
  easy_move_to_.reset();
  capture_.Reset();
}

ateam_msgs::msg::RobotMotionCommand PassReceiver::runFrame(const World & world, const Robot & robot)
{
  if (done_) {
    return runPostPass();
  }
  const ateam_geometry::Vector text_pos_offset(0.5, 0);
  if (isBallFast(world)) {
    return runPass(world, robot);
  } else if (isBallClose(world, robot)) {
    done_ = true;
    return runPostPass();
  } else if (isBallStalledAndReachable(world, robot)) {
    return capture_.runFrame(world, robot);
  } else {
    return runPrePass(world, robot);
  }
}

bool PassReceiver::isBallFast(const World & world)
{
  return ateam_geometry::norm(world.ball.vel) > 0.1;
}

bool PassReceiver::isBallClose(const World & world, const Robot & robot)
{
  const auto ball_close_to_bot = ateam_geometry::norm(world.ball.pos - robot.pos) <
    (kRobotRadius + kBallRadius + .05);
  const auto bot_close_to_target = ateam_geometry::norm(robot.pos - target_) < 1.0;
  return ball_close_to_bot && bot_close_to_target;
}

bool PassReceiver::isBallStalledAndReachable(const World & world, const Robot & robot)
{
  return ateam_geometry::norm(world.ball.vel) < 0.01 &&
         std::sqrt(CGAL::squared_distance(world.ball.pos, robot.pos)) < 1.0;
}

ateam_msgs::msg::RobotMotionCommand PassReceiver::runPrePass(
  const World & world,
  const Robot & robot)
{
  ateam_geometry::Point destination = target_;

  auto available_robots = play_helpers::getAvailableRobots(world);
  play_helpers::removeGoalie(available_robots, world);
  play_helpers::removeRobotWithId(available_robots, robot.id);
  if (!available_robots.empty()) {
    const auto likely_kicker = play_helpers::getClosestRobot(available_robots, world.ball.pos);
    const auto kicker_ball_ray = ateam_geometry::Ray(world.ball.pos, likely_kicker.pos);
    const auto projected_target_pos = kicker_ball_ray.supporting_line().projection(target_);
    const auto target_proj_vector = projected_target_pos - target_;
    // Roughly, 90% pull towards target & 10% pull towards kick line
    destination += target_proj_vector * 0.1;
  }

  easy_move_to_.setTargetPosition(destination);
  easy_move_to_.face_point(world.ball.pos);
  path_planning::PlannerOptions planner_options;
  planner_options.avoid_ball = false;
  easy_move_to_.setPlannerOptions(planner_options);
  return easy_move_to_.runFrame(robot, world);
}

ateam_msgs::msg::RobotMotionCommand PassReceiver::runPass(const World & world, const Robot & robot)
{
  const ateam_geometry::Ray ball_ray(world.ball.pos, world.ball.vel);
  const auto destination = ball_ray.supporting_line().projection(robot.pos);
  easy_move_to_.setTargetPosition(destination);
  easy_move_to_.face_point(world.ball.pos);
  path_planning::PlannerOptions planner_options;
  planner_options.avoid_ball = false;
  easy_move_to_.setPlannerOptions(planner_options);
  ateam_msgs::msg::RobotMotionCommand motion_command;
  motion_command = easy_move_to_.runFrame(robot, world);
  motion_command.dribbler_speed = 300;
  const auto dist_to_ball = ateam_geometry::norm(robot.pos - world.ball.pos);
  if (dist_to_ball < 0.5) {
    ateam_geometry::Vector robot_vel(motion_command.twist.linear.x, motion_command.twist.linear.y);
    robot_vel += ateam_geometry::normalize(world.ball.vel) * 0.6;
    motion_command.twist.linear.x = robot_vel.x();
    motion_command.twist.linear.y = robot_vel.y();
  }
  return motion_command;
}

ateam_msgs::msg::RobotMotionCommand PassReceiver::runPostPass()
{
  ateam_msgs::msg::RobotMotionCommand command;
  command.dribbler_speed = 200;
  command.twist.linear.x = 0;
  command.twist.linear.y = 0;
  command.twist.angular.z = 0;
  return command;
}

}  // namespace ateam_kenobi::skills
