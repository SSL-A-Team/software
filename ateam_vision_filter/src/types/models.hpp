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

#pragma once

#include <Eigen/Dense>

namespace Models
{

enum ModelType
{
  BALL_ROLLING_FRICTION, // Standard ball rolling over the ground
  BALL_SLIDING_FRICTION, // Ball with backspin after kick where it's sliding instead of rolling
  BALL_BOUNCE_ON_ROBOT, // Ball doing a perfectly inelastic collision
  BALL_STOP_ON_DRIBBLER,
  BALL_SLOW_KICK, // 2 m/s
  BALL_MEDIUM_KICK, // 4 m/s
  BALL_FAST_KICK, // 6 m/s

  ROBOT_NO_ACCEL,
  ROBOT_ACCEL_TOWARDS_BALL, // Move towards the ball
  ROBOT_ACCEL_AWAY_FROM_BALL // Slow down while moving towards ball
};

constexpr double dt = 1.0 / 100.0;

namespace Ball
{

// pos_x, pos_y, vel_x, vel_y, accel_x, accel_y
const Eigen::MatrixXd F =
  (Eigen::MatrixXd(6, 6) <<
  1, 0, dt, 0, 0, 0,
  0, 1, 0, dt, 0, 0,
  0, 0, 1, 0, dt, 0,
  0, 0, 0, 1, 0, dt,
  0, 0, 0, 0, 1, 0,
  0, 0, 0, 0, 0, 1).finished();

// commands
// Constant "Position" commands are constant velocity of model
// Constant "Velocity" commands are constant acceleration of model
// Constant "Acceleration" commands are constant jerk of model
const Eigen::MatrixXd B =
  (Eigen::MatrixXd(6, 6) <<
  dt, 0, 0, 0, 0, 0,
  0, dt, 0, 0, 0, 0,
  0, 0, dt, 0, 0, 0,
  0, 0, 0, dt, 0, 0,
  0, 0, 0, 0, dt, 0,
  0, 0, 0, 0, 0, dt).finished();

// Only measure position
const Eigen::MatrixXd H =
  (Eigen::MatrixXd(2, 6) <<
  1, 0, 0, 0, 0, 0,
  0, 1, 0, 0, 0, 0).finished();

const Eigen::MatrixXd Q = F;
const Eigen::MatrixXd R = F;

}

namespace Robot
{


// pos_x, pos_y, theta, vel_x, vel_y, omega, accel_x, accel_y, alpha
const Eigen::MatrixXd F =
  (Eigen::MatrixXd(9, 9) <<
  1, 0, 0, dt, 0, 0, 0, 0, 0,
  0, 1, 0, 0, dt, 0, 0, 0, 0,
  0, 0, 1, 0, 0, dt, 0, 0, 0,
  0, 0, 0, 1, 0, 0, dt, 0, 0,
  0, 0, 0, 0, 1, 0, 0, dt, 0,
  0, 0, 0, 0, 0, 1, 0, 0, dt,
  0, 0, 0, 0, 0, 0, 1, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 1, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 1).finished();

// commands
// Constant "Position" commands are constant velocity of model
// Constant "Velocity" commands are constant acceleration of model
// Constant "Acceleration" commands are constant jerk of model
const Eigen::MatrixXd B =
  (Eigen::MatrixXd(9, 9) <<
  dt, 0, 0, 0, 0, 0, 0, 0, 0,
  0, dt, 0, 0, 0, 0, 0, 0, 0,
  0, 0, dt, 0, 0, 0, 0, 0, 0,
  0, 0, 0, dt, 0, 0, 0, 0, 0,
  0, 0, 0, 0, dt, 0, 0, 0, 0,
  0, 0, 0, 0, 0, dt, 0, 0, 0,
  0, 0, 0, 0, 0, 0, dt, 0, 0,
  0, 0, 0, 0, 0, 0, 0, dt, 0,
  0, 0, 0, 0, 0, 0, 0, 0, dt).finished();

// Only measure position
const Eigen::MatrixXd H =
  (Eigen::MatrixXd(3, 9) <<
  1, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 1, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 1, 0, 0, 0, 0, 0, 0).finished();

const Eigen::MatrixXd Q = F;
const Eigen::MatrixXd R = F;

}
}
