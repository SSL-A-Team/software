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

#ifndef TYPES__ROBOT_MEASUREMENT_HPP_
#define TYPES__ROBOT_MEASUREMENT_HPP_

#include <Eigen/Dense>

struct RobotMeasurement
{
  Eigen::Vector2d position;
  double theta;

  void invert()
  {
    position *= -1.0;
    theta = angle_wrap_pi(theta + M_PI);
  }

  // TODO(Cavidano) MOVE THIS CONVERSION TO A COMMON PLACE
  double angle_wrap_2pi(double angle_rad)
  {
    double pi2 = 2 * M_PI;
    if (angle_rad >= pi2) {
      angle_rad -= pi2 * std::floor(angle_rad / pi2);
    }

    if (angle_rad < 0) {
      angle_rad += pi2 * std::ceil(std::abs(angle_rad) / pi2);
    }
    return angle_rad;
  }

  double angle_wrap_pi(double angle_rad)
  {
    if (angle_rad < -M_PI || M_PI < angle_rad) {
      return angle_wrap_2pi(angle_rad + M_PI) - M_PI;
    }
    return angle_rad;
  }
};

#endif  // TYPES__ROBOT_MEASUREMENT_HPP_
