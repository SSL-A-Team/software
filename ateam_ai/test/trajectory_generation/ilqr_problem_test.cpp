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

#include "trajectory_generation/ilqr_problem.hpp"

#include <gtest/gtest.h>
#include <iostream>

constexpr int num_samples = 100;
constexpr double dt = 0.01;
class PointMass1dProblem : public iLQRProblem<2, 1, num_samples>
{
public:
  State dynamics(const State & x_t1, const Input & u_t1, Time t) override
  {
    Eigen::Matrix<double, 2, 2> A; A << 0, 1, 0, 0;
    Eigen::Matrix<double, 2, 1> B; B << 0, 1;

    return x_t1 + dt * (A * x_t1 + B * u_t1);
  }

  Cost cost(const State & x, const Input & u, Time t) override
  {
    constexpr double final_gain = 1e6;
    if (t == num_samples - 1) {
      Eigen::Vector2d target{10, 0};
      Eigen::Vector2d weights{1, 1};
      return final_gain * (target - x).dot(target - x);
    } else {
      return dt * u.dot(u);
    }
  }
};

TEST(iLQRProblem, SamplePointMass)
{
  PointMass1dProblem problem;
  auto maybe_trajectory = problem.calculate(Eigen::Vector2d{0, 0});

  ASSERT_TRUE(maybe_trajectory.has_value());
  EXPECT_NEAR(maybe_trajectory.value().back().x(), 10, 1e-1);
  EXPECT_NEAR(maybe_trajectory.value().back().y(), 0, 1e-4);
}
