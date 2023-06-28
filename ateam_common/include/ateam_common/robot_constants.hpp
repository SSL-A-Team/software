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
constexpr double kRobotDiameter = 0.18;
constexpr double kRobotRadius = kRobotDiameter / 2;
constexpr double kRobotHeight = 0.15;

// Physical limitations - used in trajectory generation
const Eigen::Vector3d kMaxRobotVel = Eigen::Vector3d(2, 2, 0.5);
const Eigen::Vector3d kMaxRobotAccel = Eigen::Vector3d(2, 2, 0.5);

#endif // ATEAM_COMMON__ROBOT_CONSTANTS_HPP_