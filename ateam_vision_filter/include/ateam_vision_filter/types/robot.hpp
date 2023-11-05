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

#ifndef TYPES__ROBOT_HPP_
#define TYPES__ROBOT_HPP_

#include <Eigen/Dense>

class Robot
{
public:
  Eigen::Vector2d position;
  double theta;

  Eigen::Vector2d velocity;
  double omega;

  Eigen::Vector2d acceleration;
  double alpha;

  Robot()
  : position{0, 0}, theta{0}, velocity{0, 0}, omega{0}, acceleration{0, 0}, alpha{0} {}

  explicit Robot(const Eigen::Matrix<double, 9, 1> & from_state)
  : position(from_state.block(0, 0, 2, 1)), theta(from_state(2, 0)),
    velocity(from_state.block(3, 0, 2, 1)), omega(from_state(5, 0)),
    acceleration(from_state.block(6, 0, 2, 1)), alpha(from_state(8, 0)) {}
};

#endif  // TYPES__ROBOT_HPP_
