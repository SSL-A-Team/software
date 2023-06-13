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

#ifndef TYPES__CAMERA_MEASUREMENT_HPP_
#define TYPES__CAMERA_MEASUREMENT_HPP_

#include <array>
#include <vector>

#include "types/ball_measurement.hpp"
#include "types/robot_measurement.hpp"

struct CameraMeasurement
{
  std::array<std::vector<RobotMeasurement>, 16> yellow_robots;
  std::array<std::vector<RobotMeasurement>, 16> blue_robots;
  std::vector<BallMeasurement> ball;

  void invert() {
    auto invert_array = [&](auto& target_array) {
        for (auto& data : target_array) {
            data.invert();
        }
    };

    for (auto& measurement_array : yellow_robots) {
        invert_array(measurement_array);
    }

    for (auto& measurement_array : blue_robots) {
        invert_array(measurement_array);
    }

    invert_array(ball);
  }
};

#endif  // TYPES__CAMERA_MEASUREMENT_HPP_
