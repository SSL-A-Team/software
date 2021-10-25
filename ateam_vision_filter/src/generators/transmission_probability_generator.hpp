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

// Builds a transmission probability matrix
// given last robot estimate positions
// last ball estimate position
// Transmission of ball from slide to roll, etc

#include "types/ball.hpp"
#include "types/models.hpp"
#include "types/robot.hpp"

#include <Eigen/Dense>

#include <array>

class TransmissionProbabilityGenerator
{
public:
  void update(
    const std::array<Robot, 16> & blue_robots,
    const std::array<Robot, 16> & yellow_robots,
    const Ball & ball);

  double get_transmission_probability(
    const Eigen::VectorXd & possible_state,
    const Models::ModelType & from_model,
    const Models::ModelType & to_model) const;

private:
  std::array<Robot, 16> blue_robots;
  std::array<Robot, 16> yellow_robots;
  Ball ball;
};
