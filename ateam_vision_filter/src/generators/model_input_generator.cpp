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

#include "generators/model_input_generator.hpp"

#include <limits>
#include <optional>

void ModelInputGenerator::update(
  const std::array<std::optional<Robot>, 16> & blue_robots,
  const std::array<std::optional<Robot>, 16> & yellow_robots,
  const std::optional<Ball> & ball)
{
  this->blue_robots = blue_robots;
  this->yellow_robots = yellow_robots;
  this->ball = ball;
}

Eigen::VectorXd ModelInputGenerator::get_model_input(
  const Eigen::VectorXd & possible_state,
  const Models::ModelType & model_type) const
{
  // Ball
  if (model_type == Models::ModelType::BALL_ROLLING_FRICTION) {
    // Ret rolling friction deccell to 0
    Eigen::VectorXd vel = possible_state.block(2, 0, 2, 1);
    Eigen::VectorXd friction = Models::Ball::rolling_friction_accel * -vel.normalized();

    if (vel.norm() < friction.norm()) {
      friction = vel.norm() * friction.normalized();
    }

    Eigen::VectorXd output = 0.0 * possible_state;  // Easy way to get 0 vector of right size
    output.block(2, 0, 2, 1) = vel + friction;

    return output;
  } else if (model_type == Models::ModelType::BALL_SLIDING_FRICTION) {
    // Ret sliding friction deccell to 0
    Eigen::VectorXd vel = possible_state.block(2, 0, 2, 1);
    Eigen::VectorXd friction = Models::Ball::sliding_friction_accel * -vel.normalized();

    if (vel.norm() < friction.norm()) {
      friction = vel.norm() * friction.normalized();
    }

    Eigen::VectorXd output = 0.0 * possible_state;  // Easy way to get 0 vector of right size
    output.block(2, 0, 2, 1) = vel + friction;

    return output;
  } else if (model_type == Models::ModelType::BALL_BOUNCE_ON_ROBOT) {
    // Get closest robot
    Eigen::Vector2d ball_pos = possible_state.block(0, 0, 2, 1);

    std::optional<Robot> closest_robot;
    double dist = std::numeric_limits<double>::infinity();

    for (const auto & robot : blue_robots) {
      if (robot.has_value()) {
        Eigen::Vector2d robot_pos = robot.value().position;

        double test_dist = (ball_pos - robot_pos).norm();
        if (test_dist < dist) {
          closest_robot = robot;
          dist = test_dist;
        }
      }
    }
    
    for (const auto & robot : yellow_robots) {
      if (robot.has_value()) {
        Eigen::Vector2d robot_pos = robot.value().position;

        double test_dist = (ball_pos - robot_pos).norm();
        if (test_dist < dist) {
          closest_robot = robot;
          dist = test_dist;
        }
      }
    }

    // No visible robots
    if (!closest_robot.has_value()) {
      return 0.0 * possible_state;  // Easy way to get 0 vector of right size
    }

    // Figure out bounce (assuming robot is circle for now)
    // Get hit point on radius
    // Get angle to hit point
    // Reflect velocity off hitpoint
    // Remove all current velocity of ball, set to new velocity
  } else if (model_type == Models::ModelType::BALL_STOP_ON_DRIBBLER) {
    // Negate current speed like ball instantly damps on dribbler
    Eigen::Vector2d vel = possible_state.block(2, 0, 2, 1);
    Eigen::Vector2d accel = possible_state.block(4, 0, 2, 1);
    Eigen::VectorXd output = 0.0 * possible_state;  // Easy way to get 0 vector of right size
    output.block(0, 0, 2, 1) = -vel - Models::dt * accel;

    return output;
  } else if (model_type == Models::ModelType::BALL_SLOW_KICK) {
    // Facing direction of closest robot at 2 m/s
  } else if (model_type == Models::ModelType::BALL_MEDIUM_KICK) {
    // Facing direction of closest robot at 4 m/s
  } else if (model_type == Models::ModelType::BALL_FAST_KICK) {
    // Facing direction of closest robot at 6 m/s
  }

  // Robot
  if (model_type == Models::ModelType::ROBOT_NO_ACCEL) {
    return 0.0 * possible_state;  // Easy way to get 0 vector of right size
  } else if (model_type == Models::ModelType::ROBOT_ACCEL_TOWARDS_BALL) {
    // Accel at X m/s2 towards ball
  } else if (model_type == Models::ModelType::ROBOT_ACCEL_AWAY_FROM_BALL) {
    // Accel at X m/s2 away from ball
  }

  // ERROR
  return 0.0 * possible_state;
}
