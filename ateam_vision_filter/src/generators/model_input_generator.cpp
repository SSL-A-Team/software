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

void ModelInputGenerator::update(
  const std::array<Robot, 16> & blue_robots,
  const std::array<Robot, 16> & yellow_robots,
  const Ball & ball)
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
  } else if (model_type == Models::ModelType::BALL_SLIDING_FRICTION) {
    // Ret sliding friction deccell to 0
  } else if (model_type == Models::ModelType::BALL_BOUNCE_ON_ROBOT) {
    // Get closest robot
    // Figure out bounce (assuming robot is circle for now)
  } else if (model_type == Models::ModelType::BALL_STOP_ON_DRIBBLER) {
    // Negate current speed like ball instantly damps on dribbler
  } else if (model_type == Models::ModelType::BALL_SLOW_KICK) {
    // Facing direction of closest robot at 2 m/s
  } else if (model_type == Models::ModelType::BALL_MEDIUM_KICK) {
    // Facing direction of closest robot at 4 m/s
  } else if (model_type == Models::ModelType::BALL_FAST_KICK) {
    // Facing direction of closest robot at 6 m/s
  }

  // Robot
  if (model_type == Models::ModelType::ROBOT_NO_ACCEL) {
    //return Eigen::VectorXd{0, 0, 0, 0, 0, 0, 0, 0, 0};
  } else if (model_type == Models::ModelType::ROBOT_ACCEL_TOWARDS_BALL) {
    // Accel at X m/s2 towards ball
  } else if (model_type == Models::ModelType::ROBOT_ACCEL_AWAY_FROM_BALL) {
    // Accel at X m/s2 away from ball
  }

  // ERROR
  return 0.0 * possible_state;
}
