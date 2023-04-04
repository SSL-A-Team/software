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

#ifndef TRAJECTORY_GENERATION__ILQR_PROBLEM_HPP_
#define TRAJECTORY_GENERATION__ILQR_PROBLEM_HPP_

#include <Eigen/Dense>

#include <optional>
#include <iostream>

// Defines the abstract definition and solution to the iLQR problem
// https://homes.cs.washington.edu/~todorov/papers/TassaICRA14.pdf
// https://studywolf.wordpress.com/2016/02/03/the-iterative-linear-quadratic-regulator-method/
// X = num states, U = num inputs, T = num timesteps
template<int X, int U, int T>
class iLQRProblem
{
public:
  using State = Eigen::Matrix<double, X, 1>;
  using StateJacobian = Eigen::Matrix<double, X, X + U>;
  using StateJacobian_x = Eigen::Matrix<double, X, X>;
  using StateJacobian_x_T = Eigen::Matrix<double, X, X>;
  using StateJacobian_u = Eigen::Matrix<double, X, U>;
  using StateJacobian_u_T = Eigen::Matrix<double, U, X>;

  using Input = Eigen::Matrix<double, U, 1>;

  using Cost = double;

  using CostGradiant = Eigen::Matrix<double, X + U, 1>;
  using CostGradiant_x = Eigen::Matrix<double, X, 1>;
  using CostGradiant_u = Eigen::Matrix<double, U, 1>;

  using CostHessian = Eigen::Matrix<double, X + U, X + U>;
  using CostHessian_xx = Eigen::Matrix<double, X, X>;
  using CostHessian_ux = Eigen::Matrix<double, U, X>;
  using CostHessian_uu = Eigen::Matrix<double, U, U>;

  using Time = int;

  // Assume second derivative of dynamics is 0 so we don't have do tensor math
  virtual State dynamics(const State & x_t1, const Input & u_t1, Time t) = 0;
  virtual StateJacobian dynamics_jacobian(const State & x, const Input & u, Time t)
  {
    StateJacobian out;

    State d = dynamics(x, u, t);
    for (std::size_t i = 0; i < X; i++) {
      State x_eps = x;
      x_eps(i) += eps;
      out.col(i) = (dynamics(x_eps, u, t) - d) / eps;
    }

    for (std::size_t i = 0; i < U; i++) {
      Input u_eps = u;
      u_eps(i) += eps;
      out.col(i + X) = (dynamics(x, u_eps, t) - d) / eps;
    }

    return out;
  }

  virtual Cost cost(const State & x, const Input & u, Time t) = 0;
  virtual CostGradiant cost_gradiant(const State & x, const Input & u, Time t)
  {
    CostGradiant out;

    for (std::size_t i = 0; i < X; i++) {
      State x_eps_p = x;
      State x_eps_n = x;
      x_eps_p(i) += eps;
      x_eps_n(i) -= eps;
      out(i) = (cost(x_eps_p, u, t) - cost(x_eps_n, u, t)) / (2 * eps);
    }

    for (std::size_t i = 0; i < U; i++) {
      Input u_eps_p = u;
      Input u_eps_n = u;
      u_eps_p(i) += eps;
      u_eps_n(i) -= eps;
      out(i + X) = (cost(x, u_eps_p, t) - cost(x, u_eps_n, t)) / (2 * eps);
    }

    return out;
  }

  virtual CostHessian cost_hessian(const State & x, const Input & u, Time t, const CostGradiant & g)
  {
    CostHessian out;

    for (std::size_t i = 0; i < X; i++) {
      State x_eps = x;
      x_eps(i) += eps;
      out.col(i) = (cost_gradiant(x_eps, u, t) - g) / eps;
    }

    for (std::size_t i = 0; i < U; i++) {
      Input u_eps = u;
      u_eps(i) += eps;
      out.col(i + X) = (cost_gradiant(x, u_eps, t) - g) / eps;
    }

    return out;
  }

private:
  static constexpr double eps = 1e-3;  // Auto differentiation epsilon
};

#endif  // TRAJECTORY_GENERATION__ILQR_PROBLEM_HPP_
