#pragma once

#include <Eigen/Dense>

namespace Models
{

namespace Ball
{

enum ModelType {
  ROLLING_FRICTION, // Standard ball rolling over the ground
  SLIDING_FRICTION, // Ball with backspin after kick where it's sliding instead of rolling
  BOUNCE,           // Ball doing a perfectly inelastic collision
  STOP,
  SLOW_KICK,
  MEDIUM_KICK,
  FAST_KICK,
};

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