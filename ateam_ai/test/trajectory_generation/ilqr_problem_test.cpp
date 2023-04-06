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
#include "trajectory_generation/ilqr_computer.hpp"

#include <gtest/gtest.h>

constexpr int num_samples1 = 100;
constexpr double dt = 0.01;
class PointMass1dProblem : public iLQRProblem<2, 1, num_samples1>
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
    Eigen::Vector2d target{10, 0};
    Eigen::Vector2d error = x - target;
    Eigen::Matrix2d Q;
    Q << 1, 0, 0, 0;
    Eigen::Matrix<double, 1, 1> R;
    R << 0;
    if (t == num_samples1 - 1) {
      return (error.transpose() * Q * error).value();
    } else {
      return (error.transpose() * Q * error).value();
    }
  }
};

TEST(iLQRProblem, SamplePointMass)
{
  PointMass1dProblem point_mass_problem;
  iLQRParams params{
    .max_ilqr_iterations = 100,
    .max_forward_pass_iterations = 10,
    .converge_threshold = 1e-3,
    .gamma = 0.5,
    .lambda_min = 1e-6,
    .delta_zero = 2
  };
  iLQRComputer computer(point_mass_problem, params);

  auto maybe_trajectory = computer.calculate(Eigen::Vector2d{0, 0});

  ASSERT_TRUE(maybe_trajectory.has_value());
  EXPECT_NEAR(maybe_trajectory.value().back().x(), 10, 1e-1);
  EXPECT_NEAR(maybe_trajectory.value().back().y(), 0, 1e-1);
}

constexpr int num_samples2 = 100;
class NLPointMass1dProblem : public iLQRProblem<2, 1, num_samples2>
{
public:
  State dynamics(const State & x_t1, const Input & u_t1, Time t) override
  {
    Eigen::Matrix<double, 2, 2> A; A << 0, 1, 0, 0;
    Eigen::Matrix<double, 2, 1> B; B << 0, 1;

    return x_t1 + dt * (A * x_t1 + B * u_t1);
  }

  double exp_boundary(double x, double q1 = 1, double q2 = 1e-2)
  {
    return q1 * exp(q2 * x);
  }

  Cost cost(const State & x, const Input & u, Time t) override
  {
    Eigen::Vector2d target{10, 0};
    Eigen::Vector2d error = x - target;
    Eigen::Matrix2d Q;
    Q << 1, 0, 0, 0.1;
    Eigen::Matrix<double, 1, 1> R;
    R << 0;
    if (t == num_samples2 - 1) {
      return 1e3 * (error.transpose() * Q * error).value();
    } else {
      return (error.transpose() * Q * error).value() + exp_boundary(u.value());
    }
  }
};

TEST(iLQRProblem, PointMassLimitedU)
{
  NLPointMass1dProblem point_mass_problem;
  iLQRParams params{
    .max_ilqr_iterations = 100,
    .max_forward_pass_iterations = 10,
    .converge_threshold = 1e-3,
    .gamma = 0.5,
    .lambda_min = 1e-6,
    .delta_zero = 2
  };
  iLQRComputer computer(point_mass_problem, params);

  auto maybe_trajectory = computer.calculate(Eigen::Vector2d{0, 0});

  ASSERT_TRUE(maybe_trajectory.has_value());
  EXPECT_NEAR(maybe_trajectory.value().back().x(), 10, 1e-1);
  EXPECT_NEAR(maybe_trajectory.value().back().y(), 0, 1e-1);
}
