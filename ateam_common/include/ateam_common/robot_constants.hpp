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

#ifndef ATEAM_COMMON__ROBOT_CONSTANTS_HPP_
#define ATEAM_COMMON__ROBOT_CONSTANTS_HPP_

#include <Eigen/Dense>

// All of the below are in meters
// Matches the size of the robots from the SSL rulebook
constexpr double kRobotDiameter = 0.18;
constexpr double kRobotRadius = kRobotDiameter / 2;
constexpr double kRobotHeight = 0.15;

// Physical limitations - used in trajectory generation
// Units are m/s and rad/s for velocity
// m/s^2 and rad/s^2 for acceleration
// This should match what the limits are set to in the firmware.
const Eigen::Vector3d kMaxRobotVel = Eigen::Vector3d(3, 3, 18);
const Eigen::Vector3d kMaxRobotAccel = Eigen::Vector3d(3, 3, 36);

constexpr double kBallDiameter = 0.04267;
constexpr double kBallRadius = kBallDiameter / 2;

constexpr double kDefaultDribblerSpeed = 360;

#endif  // ATEAM_COMMON__ROBOT_CONSTANTS_HPP_
