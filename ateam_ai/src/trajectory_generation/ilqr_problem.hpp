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
#include <tuple>
#include <utility>

// Defines the abstract definition and solution to the iLQR problem
// https://homes.cs.washington.edu/~todorov/papers/TassaICRA14.pdf
// https://studywolf.wordpress.com/2016/02/03/the-iterative-linear-quadratic-regulator-method/
// X = num states, U = num inputs, T = num timesteps
template<int X, int U, int T>
class iLQRProblem
{
public:
  using State = Eigen::Matrix<double, X, 1>;
  using StateJacobian_x = Eigen::Matrix<double, X, X>;
  using StateJacobian_x_T = Eigen::Matrix<double, X, X>;
  using StateJacobian_u = Eigen::Matrix<double, X, U>;
  using StateJacobian_u_T = Eigen::Matrix<double, U, X>;

  using Input = Eigen::Matrix<double, U, 1>;

  using Cost = double;

  using CostGradiant_x = Eigen::Matrix<double, X, 1>;
  using CostGradiant_u = Eigen::Matrix<double, U, 1>;

  using CostHessian_xx = Eigen::Matrix<double, X, X>;
  using CostHessian_ux = Eigen::Matrix<double, U, X>;
  using CostHessian_uu = Eigen::Matrix<double, U, U>;

  using Time = int;

  // Assume second derivative of dynamics is 0 so we don't have do tensor math
  virtual State dynamics(const State & x_t1, const Input & u_t1, Time t) = 0;
  virtual std::pair<StateJacobian_x, StateJacobian_u> dynamics_jacobian(
    const State & x,
    const Input & u, Time t)
  {
    State d = dynamics(x, u, t);

    StateJacobian_x out_x;
    for (std::size_t i = 0; i < X; i++) {
      State x_eps = x;
      x_eps(i) += eps;
      out_x.col(i) = (dynamics(x_eps, u, t) - d) / eps;
    }

    StateJacobian_u out_u;
    for (std::size_t i = 0; i < U; i++) {
      Input u_eps = u;
      u_eps(i) += eps;
      out_u.col(i) = (dynamics(x, u_eps, t) - d) / eps;
    }

    return std::make_pair(out_x, out_u);
  }

  virtual Cost cost(const State & x, const Input & u, Time t) = 0;
  virtual std::pair<CostGradiant_x, CostGradiant_u> cost_gradiant(
    const State & x, const Input & u,
    Time t)
  {
    CostGradiant_x out_x;
    for (std::size_t i = 0; i < X; i++) {
      State x_eps_p = x;
      State x_eps_n = x;
      x_eps_p(i) += eps;
      x_eps_n(i) -= eps;
      out_x(i) = (cost(x_eps_p, u, t) - cost(x_eps_n, u, t)) / (2 * eps);
    }

    CostGradiant_u out_u;
    for (std::size_t i = 0; i < U; i++) {
      Input u_eps_p = u;
      Input u_eps_n = u;
      u_eps_p(i) += eps;
      u_eps_n(i) -= eps;
      out_u(i) = (cost(x, u_eps_p, t) - cost(x, u_eps_n, t)) / (2 * eps);
    }

    return std::make_pair(out_x, out_u);
  }

  virtual std::tuple<CostHessian_xx, CostHessian_ux, CostHessian_uu> cost_hessian(
    const State & x, const Input & u, Time t, const CostGradiant_x & g_x,
    const CostGradiant_u & g_u)
  {
    CostGradiant_x new_g_x;
    CostGradiant_u new_g_u;

    CostHessian_xx out_xx;
    CostHessian_ux out_ux;
    CostHessian_uu out_uu;
    for (std::size_t i = 0; i < X; i++) {
      State x_eps = x;
      x_eps(i) += eps;
      std::tie(new_g_x, new_g_u) = cost_gradiant(x_eps, u, t);
      out_ux.col(i) = (new_g_u - g_u) / eps;
      out_xx.col(i) = (new_g_x - g_x) / eps;
    }

    for (std::size_t i = 0; i < U; i++) {
      Input u_eps = u;
      u_eps(i) += eps;
      std::tie(new_g_x, new_g_u) = cost_gradiant(x, u_eps, t);
      out_uu.col(i) = (new_g_u - g_u) / eps;
    }

    return std::make_tuple(out_xx, out_ux, out_uu);
  }

private:
  static constexpr double eps = 1e-3;  // Auto differentiation epsilon
};

#endif  // TRAJECTORY_GENERATION__ILQR_PROBLEM_HPP_
