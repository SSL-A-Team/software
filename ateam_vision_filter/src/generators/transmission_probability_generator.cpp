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

#include "generators/transmission_probability_generator.hpp"

#include "generators/generator_util.hpp"

namespace ateam_vision_filter
{

void TransmissionProbabilityGenerator::update(
  const std::array<std::optional<Robot>, 16> & blue_robots,
  const std::array<std::optional<Robot>, 16> & yellow_robots,
  const std::optional<Ball> & ball)
{
  this->blue_robots = blue_robots;
  this->yellow_robots = yellow_robots;
  this->ball = ball;
}

double TransmissionProbabilityGenerator::get_transmission_probability(
  const Eigen::VectorXd & possible_state,
  const Models::ModelType & from_model,
  const Models::ModelType & to_model) const
{
  std::optional<Robot> closest_robot =
    generator_util::get_closest_robot(possible_state.block(0, 0, 2, 1), blue_robots, yellow_robots);

  // Default transition to the same model
  if (from_model == to_model) {
    return 0.85;
  }

  // Ball

  // Transition from sliding to rolling has a constant low chance
  // IRL, transition happens at 70% of the initial kick speed
  if (from_model == Models::ModelType::BALL_SLIDING_FRICTION &&
    to_model == Models::ModelType::BALL_ROLLING_FRICTION)
  {
    return 0.15;
  }

  // Transition from any to ball bound
  // If near robot, ball is moving at robot
  if (generator_util::is_near_robot(possible_state.block(0, 0, 2, 1), closest_robot) &&
    generator_util::is_moving_towards_robot(
      possible_state.block(0, 0, 2, 1),
      possible_state.block(2, 0, 2, 1), closest_robot) &&
    to_model == Models::ModelType::BALL_BOUNCE_ON_ROBOT)
  {
    return 0.15;
  }

  // Transition from any to ball stop
  // If near robot, ball is moving at robot, near mouth
  if (generator_util::is_near_robot(possible_state.block(0, 0, 2, 1), closest_robot) &&
    generator_util::is_moving_towards_robot(
      possible_state.block(0, 0, 2, 1),
      possible_state.block(2, 0, 2, 1), closest_robot) &&
    generator_util::is_in_robot_mouth(possible_state.block(0, 0, 2, 1), closest_robot) &&
    to_model == Models::ModelType::BALL_STOP_ON_DRIBBLER)
  {
    return 0.15;
  }

  // Transition from any to kicks
  // If near robot, ball is at mouth
  bool to_kick = to_model == Models::ModelType::BALL_SLOW_KICK ||
    to_model == Models::ModelType::BALL_MEDIUM_KICK ||
    to_model == Models::ModelType::BALL_FAST_KICK;
  if (generator_util::is_near_robot(possible_state.block(0, 0, 2, 1), closest_robot) &&
    generator_util::is_in_robot_mouth(possible_state.block(0, 0, 2, 1), closest_robot) &&
    to_kick)
  {
    return 0.15;
  }

  // Transition kicks to sliding friction
  // Kicks always transition to sliding friction
  bool from_kick = from_model == Models::ModelType::BALL_SLOW_KICK ||
    from_model == Models::ModelType::BALL_MEDIUM_KICK ||
    from_model == Models::ModelType::BALL_FAST_KICK;
  if (from_kick &&
    to_model == Models::ModelType::BALL_SLIDING_FRICTION)
  {
    return 0.85;
  }

  // Robot

  // Low probability of swapping to any other states
  // Must all be robot
  auto is_robot = [](auto model) {
      return model == Models::ModelType::ROBOT_NO_ACCEL ||
             model == Models::ModelType::ROBOT_ACCEL_TOWARDS_BALL ||
             model == Models::ModelType::ROBOT_ACCEL_AWAY_FROM_BALL;
    };
  if (is_robot(from_model) && is_robot(to_model) && from_model != to_model) {
    return 0.15;
  }

  // Not possible to transition
  return 0.0;
}

}  // namespace ateam_vision_filter
