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
#include "core/play_helpers/available_robots.hpp"
#include <ateam_common/robot_constants.hpp>

namespace ateam_kenobi::skills
{

PassReceiver::PassReceiver(stp::Options stp_options)
: stp::Skill(stp_options),
  easy_move_to_(createChild<play_helpers::EasyMoveTo>("EasyMoveTo"))
{}

void PassReceiver::reset()
{
  done_ = false;
  kick_happened_ = false;
  easy_move_to_.reset();
}

ateam_msgs::msg::RobotMotionCommand PassReceiver::runFrame(const World & world, const Robot & robot)
{
  easy_move_to_.setMaxVelocity(2.0);  // reset default max vel
  getPlayInfo()["Robot"] = robot.id;
  if (done_) {
    getPlayInfo()["State"] = "Done";
    return runPostPass();
  }
  const ateam_geometry::Vector text_pos_offset(0.5, 0);
  if (isBotCloseToTarget(robot) && isBallVelMatchingBotVel(world, robot) && isBallClose(world,
      robot))
  {
    done_ = true;
    getPlayInfo()["State"] = "Done";
    return runPostPass();
  } else if (hasBallBeenKicked(world) && isBallClose(world, robot) && !isBallFast(world)) {
    done_ = true;
    getPlayInfo()["State"] = "Done";
    return runPostPass();
  } else if (hasBallBeenKicked(world) && isBallFast(world) && !isBallVelMatchingBotVel(world,
      robot))
  {
    getPlayInfo()["State"] = "Pass";
    return runPass(world, robot);
  } else if (hasBallBeenKicked(world) && isBallClose(world, robot) && isBallVelMatchingBotVel(world,
      robot))
  {
    done_ = true;
    getPlayInfo()["State"] = "Post";
    return runPostPass();
  } else if (isBallStalledAndReachable(world, robot)) {
    getPlayInfo()["State"] = "Capture";
    return runApproachBall(world, robot);
  } else {
    getPlayInfo()["State"] = "Pre";
    return runPrePass(world, robot);
  }
}

bool PassReceiver::isBallFast(const World & world)
{
  return ateam_geometry::norm(world.ball.vel) > 0.1;
}

bool PassReceiver::isBallClose(const World & world, const Robot & robot)
{
  return ateam_geometry::norm(world.ball.pos - robot.pos) < (kRobotRadius + kBallRadius + .05);
}

bool PassReceiver::isBallStalledAndReachable(const World & world, const Robot & robot)
{
  return ateam_geometry::norm(world.ball.vel) < 0.01 &&
         std::sqrt(CGAL::squared_distance(world.ball.pos, robot.pos)) < 0.4;
}

bool PassReceiver::isBallVelMatchingBotVel(const World & world, const Robot & robot)
{
  return ateam_geometry::norm(world.ball.vel - robot.vel) < 0.05;
}

bool PassReceiver::isBotCloseToTarget(const Robot & robot)
{
  return ateam_geometry::norm(robot.pos - target_) < 0.5;
}

bool PassReceiver::hasBallBeenKicked(const World & world)
{
  if(ateam_geometry::norm(world.ball.vel) > 0.8 * expected_kick_speed_) {
    kick_happened_ = true;
  }
  return kick_happened_;
}

ateam_msgs::msg::RobotMotionCommand PassReceiver::runPrePass(
  const World & world,
  const Robot & robot)
{
  ateam_geometry::Point destination = target_;

  auto available_robots = play_helpers::getAvailableRobots(world);
  play_helpers::removeGoalie(available_robots, world);
  play_helpers::removeRobotWithId(available_robots, robot.id);
  const auto squared_pass_length = CGAL::squared_distance(world.ball.pos, target_);

  const auto long_pass_threshold = std::pow(1.5, 2);
  const auto excessive_error_threshold = 1.0;

  if (!available_robots.empty() && squared_pass_length > long_pass_threshold) {
    const auto likely_kicker = play_helpers::getClosestRobot(available_robots, world.ball.pos);
    const auto kicker_ball_ray = ateam_geometry::Ray(world.ball.pos, likely_kicker.pos);
    const auto projected_target_pos = kicker_ball_ray.supporting_line().projection(target_);
    const auto target_proj_vector = projected_target_pos - target_;
    if(target_proj_vector.squared_length() < excessive_error_threshold) {
      // Roughly, 90% pull towards target & 10% pull towards kick line
      destination += target_proj_vector * 0.1;
    }
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
  easy_move_to_.setTargetPosition(destination, ateam_geometry::Vector(-0.001, 0));
  easy_move_to_.face_point(world.ball.pos);
  path_planning::PlannerOptions planner_options;
  planner_options.avoid_ball = false;
  easy_move_to_.setPlannerOptions(planner_options);
  ateam_msgs::msg::RobotMotionCommand motion_command;
  motion_command = easy_move_to_.runFrame(robot, world);
  motion_command.dribbler_speed = kDefaultDribblerSpeed;
  const auto dist_to_ball = ateam_geometry::norm(robot.pos - world.ball.pos);
  const auto time_to_ball = dist_to_ball / ateam_geometry::norm(world.ball.vel);
  if (time_to_ball < 0.5) {
    const ateam_geometry::Vector robot_backwards_vec =
      ateam_geometry::directionFromAngle(robot.theta + M_PI).vector();
    const auto angle_between_vecs = ateam_geometry::ShortestAngleBetween(world.ball.vel,
        robot_backwards_vec);
    if(std::abs(angle_between_vecs) < M_PI_2) {
      ateam_geometry::Vector robot_vel(motion_command.twist.linear.x,
        motion_command.twist.linear.y);
      const auto multiplier = robot.breakbeam_ball_detected_filtered ? 0.2 : 0.9;
      robot_vel += ateam_geometry::normalize(world.ball.vel) * multiplier;
      motion_command.twist.linear.x = robot_vel.x();
      motion_command.twist.linear.y = robot_vel.y();
    }
  }
  return motion_command;
}

ateam_msgs::msg::RobotMotionCommand PassReceiver::runPostPass()
{
  ateam_msgs::msg::RobotMotionCommand command;
  command.dribbler_speed = kDefaultDribblerSpeed;
  command.twist.linear.x = 0;
  command.twist.linear.y = 0;
  command.twist.angular.z = 0;
  return command;
}

ateam_msgs::msg::RobotMotionCommand PassReceiver::runApproachBall(
  const World & world,
  const Robot & robot)
{
  const auto ball_to_bot_vector = robot.pos - world.ball.pos;
  const auto target = world.ball.pos + ateam_geometry::normalize(ball_to_bot_vector) * kRobotDiameter * 1.05;
  easy_move_to_.setTargetPosition(target);
  easy_move_to_.setMaxVelocity(1.0);
  return easy_move_to_.runFrame(robot, world);
}

}  // namespace ateam_kenobi::skills
