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

#ifndef UTIL__PID_HPP_
#define UTIL__PID_HPP_

#include <ateam_common/angle.hpp>

class PID
{
public:
  void set_kp(double kp)
  {
    this->kp = kp;
  }

  void set_ki(double ki)
  {
    this->ki = ki;
  }

  void set_kd(double kd)
  {
    this->kd = kd;
  }

  double execute(double target, double current, bool is_angle = false)
  {
    if (!is_angle) {
      double error = target - current;
      return kp * error;
    } else {
      double error = ateam_common::geometry::SignedSmallestAngleDifference(target, current);
      return kp * error;
    }
  }

private:
  double kp = 1;
  double ki = 0;
  double kd = 0;
};

#endif  // UTIL__PID_HPP_
