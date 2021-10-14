#pragma once

#include <Eigen/Dense>

namespace Models
{

enum ModelType {
  BALL_ROLLING_FRICTION, // Standard ball rolling over the ground
  BALL_SLIDING_FRICTION, // Ball with backspin after kick where it's sliding instead of rolling
  BALL_BOUNCE_ON_ROBOT,           // Ball doing a perfectly inelastic collision
  BALL_STOP_ON_DRIBBLER,
  BALL_SLOW_KICK, // 2 m/s
  BALL_MEDIUM_KICK, // 4 m/s
  BALL_FAST_KICK, // 6 m/s

  ROBOT_NO_ACCEL,
  ROBOT_ACCEL_TOWARDS_BALL, // Move towards the ball
  ROBOT_ACCEL_AWAY_FROM_BALL // Slow down while moving towards ball
};

namespace Ball
{

constexpr double dt = 1.0 / 100.0;

// pos_x, pos_y, vel_x, vel_y, accel_x, accel_y
const Eigen::MatrixXd F =
  (Eigen::MatrixXd(6, 6) <<
     1, 0, dt,  0, 0,   0,
     0, 1,  0, dt, 0,   0,
     0, 0,  1,  0, dt,  0,
     0, 0,  0,  1,  0, dt,
     0, 0,  0,  0,  1,  0,
     0, 0,  0,  0,  0,  1).finished();

// commands
// Constant "Position" commands are constant velocity of model
// Constant "Velocity" commands are constant acceleration of model
// Constant "Acceleration" commands are constant jerk of model
const Eigen::MatrixXd B =
  (Eigen::MatrixXd(6, 6) <<
     dt,  0,  0,  0,  0,  0,
      0, dt,  0,  0,  0,  0,
      0,  0, dt,  0,  0,  0,
      0,  0,  0, dt,  0,  0,
      0,  0,  0,  0, dt,  0,
      0,  0,  0,  0,  0, dt).finished();

// Only measure position
const Eigen::MatrixXd H =
  (Eigen::MatrixXd(2, 6) <<
    1, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0).finished();

}

}