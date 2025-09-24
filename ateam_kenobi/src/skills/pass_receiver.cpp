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
#include <ateam_common/robot_constants.hpp>
#include "core/play_helpers/available_robots.hpp"
#include "core/motion/motion_intent.hpp"
#include "core/motion/frame_conversions.hpp"

namespace ateam_kenobi::skills
{

PassReceiver::PassReceiver(stp::Options stp_options)
: stp::Skill(stp_options)
{}

void PassReceiver::reset()
{
  done_ = false;
  kick_happened_ = false;
}

RobotCommand PassReceiver::runFrame(const World & world, const Robot & robot)
{
  getPlayInfo()["Robot"] = robot.id;
  if (done_) {
    getPlayInfo()["State"] = "Done";
    return runPostPass();
  }
  const ateam_geometry::Vector text_pos_offset(0.5, 0);
  const auto bot_close_to_target = isBotCloseToTarget(robot);
  const auto ball_vel_matching_bot_vel = isBallVelMatchingBotVel(world, robot);
  const auto ball_close = isBallClose(world, robot);
  const auto ball_fast = isBallFast(world);
  const auto ball_was_kicked = hasBallBeenKicked(world);
  const auto ball_stalled_and_reachable = isBallStalledAndReachable(world, robot);

  RobotCommand command;

  if (bot_close_to_target && ball_vel_matching_bot_vel && ball_close) {
    done_ = true;
    getPlayInfo()["State"] = "Done";
    command = runPostPass();
  } else if (ball_was_kicked && ball_close && !ball_fast) {
    done_ = true;
    getPlayInfo()["State"] = "Done";
    command = runPostPass();
  } else if (ball_was_kicked && ball_fast && !ball_vel_matching_bot_vel) {
    getPlayInfo()["State"] = "Pass";
    command = runPass(world, robot);
  } else if (ball_was_kicked && ball_close && ball_vel_matching_bot_vel) {
    done_ = true;
    getPlayInfo()["State"] = "Post";
    command = runPostPass();
  } else if (ball_stalled_and_reachable) {
    getPlayInfo()["State"] = "Capture";
    command = runApproachBall(world, robot);
  } else {
    getPlayInfo()["State"] = "Pre";
    command = runPrePass(world, robot);
  }

  command.motion_intent.motion_options.max_velocity = 2.0;

  return command;
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

RobotCommand PassReceiver::runPrePass(
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

  RobotCommand command;
  command.motion_intent.linear = motion::intents::linear::PositionIntent{destination};
  command.motion_intent.angular = motion::intents::angular::FacingIntent{world.ball.pos};
  command.motion_intent.planner_options.avoid_ball = false;
  return command;
}

RobotCommand PassReceiver::runPass(const World & world, const Robot & robot)
{
  RobotCommand command;
  const ateam_geometry::Ray ball_ray(world.ball.pos, world.ball.vel);
  const auto destination = ball_ray.supporting_line().projection(robot.pos);
  command.motion_intent.linear = motion::intents::linear::PositionIntent{destination};
  command.motion_intent.angular = motion::intents::angular::FacingIntent{world.ball.pos};
  command.motion_intent.motion_options.max_deceleration = 4.0;
  command.motion_intent.planner_options.avoid_ball = false;

  command.dribbler_speed = kDefaultDribblerSpeed * 1.2;
  const auto dist_to_ball = ateam_geometry::norm(robot.pos - world.ball.pos);
  const auto time_to_ball = dist_to_ball / ateam_geometry::norm(world.ball.vel);
  if (time_to_ball < 0.5) {
    const ateam_geometry::Vector robot_backwards_vec =
      ateam_geometry::directionFromAngle(robot.theta + M_PI).vector();
    const auto angle_between_vecs = ateam_geometry::ShortestAngleBetween(world.ball.vel,
        robot_backwards_vec);
    if(std::abs(angle_between_vecs) < M_PI_2) {
      command.motion_intent.callback = [](motion::BodyVelocity plan_velocity,
        const path_planning::Path &, const Robot & robot,
        const World & world) -> motion::BodyVelocity {
          const auto multiplier = robot.breakbeam_ball_detected_filtered ? 0.2 : 0.6;
          plan_velocity.linear +=
            motion::WorldToLocalFrame(ateam_geometry::normalize(world.ball.vel) * multiplier,
            robot);
          return plan_velocity;
        };
    }
  }
  return command;
}

RobotCommand PassReceiver::runPostPass()
{
  RobotCommand command;
  command.dribbler_speed = kDefaultDribblerSpeed;
  return command;
}

RobotCommand PassReceiver::runApproachBall(
  const World & world,
  const Robot & robot)
{
  const auto ball_to_bot_vector = robot.pos - world.ball.pos;
  const auto target = world.ball.pos + ateam_geometry::normalize(ball_to_bot_vector) *
    kRobotDiameter * 1.05;
  RobotCommand command;
  command.motion_intent.linear = motion::intents::linear::PositionIntent{target};
  command.motion_intent.motion_options.max_velocity = 1.0;
  return command;
}

}  // namespace ateam_kenobi::skills
