// Copyright 2025 A Team
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

#ifndef PLAYS__TEST_PLAYS__TEST_PIVOT_PLAY_HPP_
#define PLAYS__TEST_PLAYS__TEST_PIVOT_PLAY_HPP_

#include <angles/angles.h>
#include "core/stp/play.hpp"
#include "core/play_helpers/available_robots.hpp"

namespace ateam_kenobi::plays
{

class TestPivotPlay : public stp::Play
{
public:
  static constexpr const char * kPlayName = "TestPivotPlay";

  explicit TestPivotPlay(stp::Options stp_options)
  : stp::Play(kPlayName, stp_options)
  {
  }

  void enter() override
  {
    target_angle_ = 0.0;
  }

  std::array<std::optional<RobotCommand>,
    16> runFrame(const World & world) override
  {
    std::array<std::optional<RobotCommand>, 16> motion_commands;
    const auto & robots = play_helpers::getAvailableRobots(world);
    if (robots.empty()) {
      return {};
    }
    const auto robot = robots.front();

    auto & play_info = getPlayInfo();
    play_info["Robot"] = robot.id;

    const auto angle_error = angles::shortest_angular_distance(robot.theta, target_angle_);

    const auto at_target = std::abs(angle_error) < target_tolerance_;

    play_info["Angle"] = robot.theta;
    play_info["Angle Error"] = angle_error;

    play_info["At Target"] = at_target ? "yes" : "no";
    play_info["Target Angle"] = target_angle_;

    if(at_target) {
      if(!prev_at_target_) {
        arrival_time_ = world.current_time;
      }
      const auto seconds_at_target =
        std::chrono::duration_cast<std::chrono::duration<double>>(world.current_time -
          arrival_time_).count();
      play_info["Time At Target"] = seconds_at_target;
      if(seconds_at_target > wait_time_) {
        target_angle_ += target_step_;
        target_angle_ = angles::normalize_angle(target_angle_);
      }
      motion_commands[robot.id] = RobotCommand();
    } else {
      play_info["Time At Target"] = 0.0;
      motion_commands[robot.id] = getPivotCommand(robot, angle_error);
    }
    prev_at_target_ = at_target;

    play_info["Y Cmd"] = motion_commands[robot.id]->twist.linear.y;
    play_info["Omega Cmd"] = motion_commands[robot.id]->twist.angular.z;

    return motion_commands;
  }

private:
  static constexpr double target_step_ = M_PI / 2.0;
  static constexpr double wait_time_ = 2.0;  // s
  static constexpr double target_tolerance_ = 0.1;  // rad
  static constexpr double pivot_speed_ = 2.0;  // rad/s
  static constexpr double pivot_accel_ = 2.0;  // rad/s^2
  double target_angle_ = 0.0;
  bool prev_at_target_ = false;
  std::chrono::steady_clock::time_point arrival_time_;


  RobotCommand getPivotCommand(const Robot & robot, double angle_error)
  {
    RobotCommand command;

    const double vel = robot.prev_command_omega;
    const double dt = 0.01;

    getPlayInfo()["Prev Cmd Omega"] = vel;

    double deceleration_to_reach_target = (vel * vel) / (2 * angle_error);

    getPlayInfo()["Target Decel"] = deceleration_to_reach_target;

    // Cruise
    double trapezoidal_vel = std::copysign(pivot_speed_, angle_error);
    const double error_direction = std::copysign(1, angle_error);
    const double decel_direction = std::copysign(1, vel * angle_error);

    // Decelerate to target velocity
    if (decel_direction > 0 && abs(deceleration_to_reach_target) > pivot_accel_ * 0.95) {
      trapezoidal_vel = vel - (error_direction * deceleration_to_reach_target * dt);
      getPlayInfo()["State"] = "Decel";
    // Accelerate to speed
    } else if (abs(vel) < pivot_speed_) {
      trapezoidal_vel = vel + (error_direction * pivot_accel_ * dt);
      getPlayInfo()["State"] = "Accel";
    } else {
      getPlayInfo()["State"] = "Cruise";
    }

    getPlayInfo()["Trap Vel"] = trapezoidal_vel;

    const auto min_angular_vel = 1.0;
    if (abs(trapezoidal_vel) < min_angular_vel) {
      trapezoidal_vel = std::copysign(min_angular_vel, angle_error);
    }

    command.twist.angular.z = std::clamp(trapezoidal_vel, -pivot_speed_, pivot_speed_);

    /* rotate in a circle with diameter 0.0427 + 0.18 = 0.2227 (This might be tunable to use 8cm for
    * real robots)
    * circumference of 0.6996 meters in a full rotation.
    * Calculate m/rev * rev/s to get linear m/s
    */
    // double diameter = kBallDiameter + kRobotDiameter;
    double diameter = 2 * .095;
    double circumference = M_PI * diameter;
    double velocity = circumference * (command.twist.angular.z / (2 * M_PI));

    command.twist.linear.x = 0.0;
    command.twist.linear.y = -velocity;
    command.twist_frame = RobotCommand::FRAME_BODY;
    return command;
  }
};

}  // namespace ateam_kenobi::plays

#endif  // PLAYS__TEST_PLAYS__TEST_PIVOT_PLAY_HPP_
