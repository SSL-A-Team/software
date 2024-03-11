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

#include <optional>

#include "generators/generator_util.hpp"

namespace ateam_vision_filter
{

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

    // Try to set output position to be if the accel happened the entire frame
    // Try to set output velocity to be decreased by friction
    // Try to set output acceleration to take into account friction
    Eigen::VectorXd output = Eigen::VectorXd::Zero(possible_state.innerSize());
    output.block(0, 0, 2, 1) = Models::dt * Models::dt / 2.0 * friction;
    output.block(2, 0, 2, 1) = Models::dt * friction;
    output.block(4, 0, 2, 1) = friction;

    return output;
  } else if (model_type == Models::ModelType::BALL_SLIDING_FRICTION) {
    // Ret sliding friction deccell to 0
    Eigen::VectorXd vel = possible_state.block(2, 0, 2, 1);
    Eigen::VectorXd friction = Models::Ball::sliding_friction_accel * -vel.normalized();

    if (vel.norm() < friction.norm()) {
      friction = vel.norm() * friction.normalized();
    }

    // Try to set output position to be if the accel happened the entire frame
    // Try to set output velocity to be decreased by friction
    // Try to set output acceleration to take into account friction
    Eigen::VectorXd output = Eigen::VectorXd::Zero(possible_state.innerSize());
    output.block(0, 0, 2, 1) = Models::dt * Models::dt / 2.0 * friction;
    output.block(2, 0, 2, 1) = Models::dt * friction;
    output.block(4, 0, 2, 1) = friction;

    return output;
  } else if (model_type == Models::ModelType::BALL_BOUNCE_ON_ROBOT) {
    // Get closest robot
    Eigen::Vector2d ball_pos = possible_state.block(0, 0, 2, 1);
    Eigen::Vector2d ball_vel = possible_state.block(2, 0, 2, 1);

    std::optional<Robot> closest_robot = generator_util::get_closest_robot(
      ball_pos, blue_robots,
      yellow_robots);

    // No visible robots
    if (!closest_robot.has_value()) {
      return Eigen::VectorXd::Zero(possible_state.innerSize());
    }

    // https://math.stackexchange.com/questions/4236016/how-to-calculate-two-circles-bouncing-off-of-each-other
    // Get hit point on radius
    // Get angle to hit point
    // Reflect velocity off hitpoint
    // Remove all current velocity of ball, set to new velocity
    Eigen::Vector2d robot_pos = closest_robot.value().position;
    Eigen::Vector2d robot_vel = closest_robot.value().velocity;

    Eigen::Vector2d n = (ball_pos - robot_pos).normalized();
    Eigen::Vector2d output_ball_vel = ball_vel - n.dot(ball_vel - robot_vel) * n;

    Eigen::VectorXd output = Eigen::VectorXd::Zero(possible_state.innerSize());
    output.block(0, 0, 2, 1) = -Models::dt * ball_vel + Models::dt * output_ball_vel;
    output.block(2, 0, 2, 1) = -ball_vel + output_ball_vel;
    output.block(4, 0, 2, 1) = Eigen::Vector2d::Zero();

    return output;
  } else if (model_type == Models::ModelType::BALL_STOP_ON_DRIBBLER) {
    // Negate current speed like ball instantly damps on dribbler
    Eigen::Vector2d vel = possible_state.block(2, 0, 2, 1);
    Eigen::Vector2d accel = possible_state.block(4, 0, 2, 1);

    // Try to set output position to be if the 0 vel/accel happened the entire frame
    // Try to set output velocity to be 0
    // Try to set output acceleration to be 0
    Eigen::VectorXd output = Eigen::VectorXd::Zero(possible_state.innerSize());
    output.block(0, 0, 2, 1) = -Models::dt * vel - Models::dt * Models::dt / 2.0 * accel;
    output.block(2, 0, 2, 1) = -Models::dt * accel;
    output.block(4, 0, 2, 1) = -accel;

    return output;
  } else if (model_type == Models::ModelType::BALL_SLOW_KICK) {
    // Facing direction of closest robot at 2 m/s
    return get_output_with_kick_at_speed(possible_state, 2.0);
  } else if (model_type == Models::ModelType::BALL_MEDIUM_KICK) {
    // Facing direction of closest robot at 4 m/s
    return get_output_with_kick_at_speed(possible_state, 4.0);
  } else if (model_type == Models::ModelType::BALL_FAST_KICK) {
    // Facing direction of closest robot at 6 m/s
    return get_output_with_kick_at_speed(possible_state, 6.0);
  }

  // Robot
  if (model_type == Models::ModelType::ROBOT_NO_ACCEL) {
    return Eigen::VectorXd::Zero(possible_state.innerSize());
  } else if (model_type == Models::ModelType::ROBOT_ACCEL_TOWARDS_BALL) {
    // Accel at X m/s2 towards ball
    Eigen::Vector2d robot_pos = possible_state.block(0, 0, 2, 1);
    Eigen::Vector2d robot_velocity = possible_state.block(3, 0, 2, 1);

    // Make sure we actually have an idea where the ball is
    if (!ball.has_value()) {
      return Eigen::VectorXd::Zero(possible_state.innerSize());
    }

    Eigen::Vector2d robot_to_ball_vector = (ball.value().position - robot_pos).normalized();
    Eigen::Vector2d new_velocity = robot_velocity + Models::dt * Models::Robot::max_acceleration *
      robot_to_ball_vector;
    if (new_velocity.norm() > Models::Robot::max_speed) {
      new_velocity = Models::Robot::max_speed * new_velocity.normalized();
    }

    Eigen::Vector2d realized_acceleration = (new_velocity - robot_velocity) / Models::dt;

    // Try to set output position to be if the accel happened the entire frame
    // Try to set output velocity to be if the accel happened the entire frame
    // Try to set output acceleration add in accel
    Eigen::VectorXd output = Eigen::VectorXd::Zero(possible_state.innerSize());
    output.block(0, 0, 2, 1) = Models::dt * Models::dt / 2.0 * realized_acceleration;
    output.block(3, 0, 2, 1) = Models::dt * realized_acceleration;
    output.block(6, 0, 2, 1) = realized_acceleration;

    return output;
  } else if (model_type == Models::ModelType::ROBOT_ACCEL_AWAY_FROM_BALL) {
    // Accel at X m/s2 away from ball
    // Accel at X m/s2 towards ball
    Eigen::Vector2d robot_pos = possible_state.block(0, 0, 2, 1);
    Eigen::Vector2d robot_velocity = possible_state.block(3, 0, 2, 1);

    // Make sure we actually have an idea where the ball is
    if (!ball.has_value()) {
      return Eigen::VectorXd::Zero(possible_state.innerSize());
    }

    Eigen::Vector2d robot_to_ball_vector = (ball.value().position - robot_pos).normalized();
    Eigen::Vector2d new_velocity = robot_velocity - Models::dt * Models::Robot::max_acceleration *
      robot_to_ball_vector;
    if (new_velocity.norm() > Models::Robot::max_speed) {
      new_velocity = Models::Robot::max_speed * new_velocity.normalized();
    }

    Eigen::Vector2d realized_acceleration = (new_velocity - robot_velocity) / Models::dt;

    // Try to set output position to be if the accel happened the entire frame
    // Try to set output velocity to be if the accel happened the entire frame
    // Try to set output acceleration add in accel
    Eigen::VectorXd output = Eigen::VectorXd::Zero(possible_state.innerSize());
    output.block(0, 0, 2, 1) = Models::dt * Models::dt / 2.0 * realized_acceleration;
    output.block(3, 0, 2, 1) = Models::dt * realized_acceleration;
    output.block(6, 0, 2, 1) = realized_acceleration;

    return output;
  }

  // ERROR
  return Eigen::VectorXd::Zero(possible_state.innerSize());
}

Eigen::VectorXd ModelInputGenerator::get_output_with_kick_at_speed(
  const Eigen::VectorXd & possible_state, const double kick_speed) const
{
  // Facing direction of closest robot at X m/s
  Eigen::Vector2d ball_pos = possible_state.block(0, 0, 2, 1);
  Eigen::Vector2d ball_velocity = possible_state.block(2, 0, 2, 1);
  Eigen::Vector2d ball_accel = possible_state.block(4, 0, 2, 1);

  std::optional<Robot> closest_robot = generator_util::get_closest_robot(
    ball_pos, blue_robots,
    yellow_robots);

  // No visible robots
  if (!closest_robot.has_value()) {
    return Eigen::VectorXd::Zero(possible_state.innerSize());
  }

  double theta = closest_robot.value().theta;
  Eigen::Vector2d kick_velocity = kick_speed * Eigen::Vector2d{std::cos(theta), std::sin(theta)};

  // Try to set output position to be if the kick_velocity happened the entire frame
  // Try to set output velocity to be kick_velocity
  // Try to set output acceleration to be 0
  Eigen::VectorXd output = Eigen::VectorXd::Zero(possible_state.innerSize());
  output.block(
    0, 0, 2,
    1) = -Models::dt * ball_velocity - Models::dt * Models::dt / 2.0 * ball_accel + Models::dt *
    kick_velocity;
  output.block(2, 0, 2, 1) = -Models::dt * ball_accel + kick_velocity;
  output.block(4, 0, 2, 1) = -ball_accel;

  return output;
}

}  // namespace ateam_vision_filter
