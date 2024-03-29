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

#ifndef TYPES__BALL_HPP_
#define TYPES__BALL_HPP_

#include <Eigen/Dense>

namespace ateam_vision_filter
{

class Ball
{
public:
  Eigen::Vector2d position;

  Eigen::Vector2d velocity;

  Eigen::Vector2d acceleration;

  Ball()
  : position{0, 0}, velocity{0, 0}, acceleration{0, 0} {}

  explicit Ball(const Eigen::Matrix<double, 6, 1> & from_state)
  : position(from_state.block(0, 0, 2, 1)),
    velocity(from_state.block(2, 0, 2, 1)),
    acceleration(from_state.block(4, 0, 2, 1)) {}
};

}  // namespace ateam_vision_filter

#endif  // TYPES__BALL_HPP_
