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

#ifndef TYPES__MODELS_HPP_
#define TYPES__MODELS_HPP_

#include <Eigen/Dense>

namespace Models
{

enum ModelType
{
  TEST_EMPTY_MODEL,

  BALL_ROLLING_FRICTION,  // Standard ball rolling over the ground
  BALL_SLIDING_FRICTION,  // Ball with backspin after kick where it's sliding instead of rolling
  BALL_BOUNCE_ON_ROBOT,  // Ball doing a perfectly inelastic collision
  BALL_STOP_ON_DRIBBLER,
  BALL_SLOW_KICK,  // 2 m/s
  BALL_MEDIUM_KICK,  // 4 m/s
  BALL_FAST_KICK,  // 6 m/s

  ROBOT_NO_ACCEL,
  ROBOT_ACCEL_TOWARDS_BALL,  // Move towards the ball
  ROBOT_ACCEL_AWAY_FROM_BALL  // Slow down while moving towards ball
};

constexpr double dt = 1.0 / 100.0;

namespace Ball
{

const double rolling_friction_accel = 0.1;  // m/s^2 decel of rolling friction
const double sliding_friction_accel = 0.1;  // m/s^2 decel of sliding friction

// pos_x, pos_y, vel_x, vel_y, accel_x, accel_y
const double dt22 = dt * dt / 2;  // accel -> position
const Eigen::MatrixXd F =
  (Eigen::MatrixXd(6, 6) <<
  1, 0, dt, 0, dt22, 0,
  0, 1, 0, dt, 0, dt22,
  0, 0, 1, 0, dt, 0,
  0, 0, 0, 1, 0, dt,
  0, 0, 0, 0, 1, 0,
  0, 0, 0, 0, 0, 1).finished();

// commands
// Note that this is x = x + B * u, so make sure dt's are in the right space
const Eigen::MatrixXd B =
  (Eigen::MatrixXd(6, 6) <<
  1, 0, 0, 0, 0, 0,
  0, 1, 0, 0, 0, 0,
  0, 0, 1, 0, 0, 0,
  0, 0, 0, 1, 0, 0,
  0, 0, 0, 0, 1, 0,
  0, 0, 0, 0, 0, 1).finished();

// Only measure position
const Eigen::MatrixXd H =
  (Eigen::MatrixXd(2, 6) <<
  1, 0, 0, 0, 0, 0,
  0, 1, 0, 0, 0, 0).finished();

const double dt4 = dt * dt * dt * dt / 4.0;
const double dt3 = dt * dt * dt / 2.0;
const double dt2 = dt * dt / 2.0;
const double sigma_alpha_squared = 0.1;  // Acceleration std dev
const Eigen::MatrixXd Q =
  (Eigen::MatrixXd(6, 6) <<
  dt4, 0, dt3, 0, dt2, 0,
  0, dt4, 0, dt3, 0, dt2,
  dt3, 0, dt * dt, 0, dt, 0,
  0, dt3, 0, dt * dt, 0, dt,
  dt2, 0, dt, 0, 1, 0,
  0, dt2, 0, dt, 0, 1).finished() * sigma_alpha_squared;

const double sigma_pos_squared = 0.1;  // Position measurement error
const Eigen::MatrixXd R =
  (Eigen::MatrixXd(2, 2) <<
  sigma_pos_squared, 0,
  0, sigma_pos_squared).finished();

}  // namespace Ball

namespace Robot
{

const double mouth_half_angle = 45.0 * 3.14 / 180.0;  // Angle between forward and edge of the mouth

const double max_speed = 6;
const double max_acceleration = 12;

// pos_x, pos_y, theta, vel_x, vel_y, omega, accel_x, accel_y, alpha
const double dt22 = dt * dt / 2;  // accel -> position
const Eigen::MatrixXd F =
  (Eigen::MatrixXd(9, 9) <<
  1, 0, 0, dt, 0, 0, dt22, 0, 0,
  0, 1, 0, 0, dt, 0, 0, dt22, 0,
  0, 0, 1, 0, 0, dt, 0, 0, dt22,
  0, 0, 0, 1, 0, 0, dt, 0, 0,
  0, 0, 0, 0, 1, 0, 0, dt, 0,
  0, 0, 0, 0, 0, 1, 0, 0, dt,
  0, 0, 0, 0, 0, 0, 1, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 1, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 1).finished();

// commands
// Note that this is x = x + B * u, so make sure dt's are in the right space
const Eigen::MatrixXd B =
  (Eigen::MatrixXd(9, 9) <<
  1, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 1, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 1, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 1, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 1, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 1, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 1, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 1, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 1).finished();

// Only measure position
const Eigen::MatrixXd H =
  (Eigen::MatrixXd(3, 9) <<
  1, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 1, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 1, 0, 0, 0, 0, 0, 0).finished();

const double dt4 = dt * dt * dt * dt / 4.0;
const double dt3 = dt * dt * dt / 2.0;
const double dt2 = dt * dt / 2.0;
const double sigma_linear_alpha_squared = .1;  // Linear acceleration std dev
const double sigma_angular_alpha_squared = .1;  // Angular acceleration std dev
// Split into linear and angular portion
const Eigen::MatrixXd Q =
  (Eigen::MatrixXd(9, 9) <<
  dt4, 0, 0, dt3, 0, 0, dt2, 0, 0,
  0, dt4, 0, 0, dt3, 0, 0, dt2, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0,
  dt3, 0, 0, dt * dt, 0, 0, dt, 0, 0,
  0, dt3, 0, 0, dt * dt, 0, 0, dt, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0,
  dt2, 0, 0, dt, 0, 0, 1, 0, 0,
  0, dt2, 0, 0, dt, 0, 0, 1, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0).finished() * sigma_linear_alpha_squared +
  (Eigen::MatrixXd(9, 9) <<
  0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, dt4, 0, 0, dt3, 0, 0, dt2,
  0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, dt3, 0, 0, dt * dt, 0, 0, dt,
  0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, dt2, 0, 0, dt, 0, 0, 1).finished() * sigma_angular_alpha_squared;

const double sigma_pos_squared = 0.1;  // Position measurement error
const double sigma_theta_squared = 0.1;  // Angular heading measurement error
const Eigen::MatrixXd R =
  (Eigen::MatrixXd(3, 3) <<
  sigma_pos_squared, 0, 0,
  0, sigma_pos_squared, 0,
  0, 0, sigma_theta_squared).finished();

}  // namespace Robot
}  // namespace Models

#endif  // TYPES__MODELS_HPP_
