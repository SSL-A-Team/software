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

#ifndef GENERATORS__MODEL_INPUT_GENERATOR_HPP_
#define GENERATORS__MODEL_INPUT_GENERATOR_HPP_

#include <Eigen/Dense>

#include <optional>
#include <array>

#include "types/ball.hpp"
#include "types/models.hpp"
#include "types/robot.hpp"

namespace ateam_vision_filter
{

class ModelInputGenerator
{
public:
  void update(
    const std::array<std::optional<Robot>, 16> & blue_robots,
    const std::array<std::optional<Robot>, 16> & yellow_robots,
    const std::optional<Ball> & ball);

  Eigen::VectorXd get_model_input(
    const Eigen::VectorXd & possible_state,
    const Models::ModelType & model_type) const;

private:
  std::array<std::optional<Robot>, 16> blue_robots;
  std::array<std::optional<Robot>, 16> yellow_robots;
  std::optional<Ball> ball;

  /**
   * @return Closest robot to the position given (if one exists)
   */
  std::optional<Robot> get_closest_robot(const Eigen::Vector2d & position) const;

  Eigen::VectorXd get_output_with_kick_at_speed(
    const Eigen::VectorXd & possible_state,
    const double kick_speed) const;
};

}  // namespace ateam_vision_filter

#endif  // GENERATORS__MODEL_INPUT_GENERATOR_HPP_
