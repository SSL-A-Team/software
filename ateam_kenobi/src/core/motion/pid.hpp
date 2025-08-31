// Copyright 2025 A Team
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

#ifndef CORE__MOTION__PID_HPP_
#define CORE__MOTION__PID_HPP_

#include <algorithm>
#include <cmath>

class PID
{
public:
  PID() = default;
  PID(double p, double i, double d, double i_max = 0, double i_min = 0)
  {
    set_gains(p, i, d, i_max, i_min);
  }


  double compute_command(const double error, const double dt)
  {
    if (std::isnan(error)) {
      prev_error_ = 0;
      return 0;
    }

    integral_ += error * dt;
    if (use_antiwindup_) {
      integral_ = std::clamp(integral_, i_min_, i_max_);
    }

    double derivative = (error - prev_error_) / dt;

    double command = (kp_ * error) + (ki_ * integral_) + (kd_ * derivative);
    prev_error_ = error;

    return command;
  }

  void set_gains(double p, double i, double d, double i_max = 0, double i_min = 0)
  {
    kp_ = p;
    ki_ = i;
    kd_ = d;

    if (i_min == 0 && i_max == 0) {
      use_antiwindup_ = false;
    } else {
      use_antiwindup_ = true;
      i_min_ = i_min;
      i_max_ = i_max;
    }

    integral_ = 0;
  }

private:
  double kp_ = 0;
  double ki_ = 0;
  double kd_ = 0;

  bool use_antiwindup_ = false;
  double i_max_ = 0;
  double i_min_ = 0;

  double prev_error_ = 0;
  double integral_ = 0;
};

#endif  // CORE__MOTION__PID_HPP_
