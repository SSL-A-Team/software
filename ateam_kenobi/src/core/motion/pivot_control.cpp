// Copyright 2026 A Team
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

#include "pivot_control.hpp"

namespace ateam_kenobi::motion
{

MotionCommand PivotAtVelocity(const intents::PivotVelocity & intent)
{
  MotionCommand command;
  command.control_mode = ControlMode::LocalVelocity;
  command.velocity.x = 0.0;
  const auto circumference = M_PI * 2.0 * intent.radius;
  command.velocity.y = -1.0 * circumference * (intent.angular_velocity / (2.0 * M_PI));
  command.velocity.theta = intent.angular_velocity;
  command.limit_vel_linear = intent.limits.linear_velocity;
  command.limit_vel_angular = intent.limits.angular_velocity;
  command.limit_acc_linear = intent.limits.linear_acceleration;
  command.limit_acc_angular = intent.limits.angular_acceleration;
  return command;
}

MotionCommand PivotToHeading(const intents::PivotHeading & intent, const Robot & robot)
{
  (void)intent;
  (void)robot;  
  const auto angle_error = angles::shortest_angular_distance(robot.theta, intent.target_heading);
  const auto dt = 0.01;
  const auto prev_vel = robot.omega;
  auto pivot_speed = 2.8;  // rad/s
  if(intent.limits.angular_velocity > 1e-6) {
    pivot_speed = intent.limits.angular_velocity;
  }
  auto pivot_accel = 3.5;  // rad/s^2
  if(intent.limits.angular_acceleration > 1e-6) {
    pivot_accel = intent.limits.angular_acceleration;
  }
  const auto deceleration_to_reach_target = (prev_vel * prev_vel) / (2.0 * angle_error);
  // Cruise
  auto trapezoidal_vel = std::copysign(pivot_speed, angle_error);
  const auto error_direction = std::copysign(1, angle_error);
  const auto decel_direction = std::copysign(1, prev_vel * angle_error);
  // Decelerate to target velocity
  if (decel_direction > 0 && abs(deceleration_to_reach_target) > pivot_accel * 0.95) {
    trapezoidal_vel = prev_vel - (error_direction * deceleration_to_reach_target * dt);

  // Accelerate to speed
  } else if (abs(prev_vel) < pivot_speed) {
    trapezoidal_vel = prev_vel + (error_direction * pivot_accel * dt);
  }
  const auto min_angular_vel = 0.5;
  if (abs(trapezoidal_vel) < min_angular_vel) {
    trapezoidal_vel = std::copysign(min_angular_vel, angle_error);
  }
  const auto angular_vel = std::clamp(trapezoidal_vel, -pivot_speed, pivot_speed);
  intents::PivotVelocity inner_intent;
  inner_intent.angular_velocity = angular_vel;
  inner_intent.radius = intent.radius;
  inner_intent.limits = intent.limits;
  return PivotAtVelocity(inner_intent);
}

}  // namespace ateam_kenobi::motion
