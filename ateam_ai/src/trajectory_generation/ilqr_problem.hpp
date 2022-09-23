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
  using Input = Eigen::Matrix<double, U, 1>;
  using Jacobian = Eigen::Matrix<double, X, X + U>;
  using Cost = double;
  using Gradiant = Eigen::Matrix<double, X + U, 1>;
  using Hessian = Eigen::Matrix<double, X + U, X + U>;
  using Time = int;

  using Trajectory = std::array<State, T>;
  using Actions = std::array<Input, T>;

  using Feedforwards = std::array<Input, T>;
  using Feedbacks = std::array<Eigen::Matrix<double, U, X>, T>;

  // Assume second derivative of dynamics is 0 so we don't have do tensor math
  virtual State dynamics(const State & x_t1, const Input & u_t1, Time t) = 0;
  virtual Jacobian jacobian(const State & x, const Input & u, Time t)
  {
    Jacobian out;

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
  virtual Gradiant gradiant(const State & x, const Input & u, Time t)
  {
    Gradiant out;

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

  virtual Hessian hessian(const State & x, const Input & u, Time t, const Gradiant & g)
  {
    Hessian out;

    for (std::size_t i = 0; i < X; i++) {
      State x_eps = x;
      x_eps(i) += eps;
      out.col(i) = (gradiant(x_eps, u, t) - g) / eps;
    }

    for (std::size_t i = 0; i < U; i++) {
      Input u_eps = u;
      u_eps(i) += eps;
      out.col(i + X) = (gradiant(x, u_eps, t) - g) / eps;
    }

    return out;
  }

  Trajectory calculate(const State & initial_state)
  {
    forward_rollout(initial_state);
    backward_pass();

    return trajectory;
  }

private:
  using Jacobians = std::array<Jacobian, T>;
  using Hessians = std::array<Hessian, T>;
  using Gradiants = std::array<Gradiant, T>;


  void forward_rollout(const State & initial_state)
  {
    // Step 1: Forward Rollout
    trajectory.front() = initial_state;
    actions.front().setZero();
    overall_cost = cost(trajectory.at(0), actions.at(0), 0);

    for (Time t = 1; t < T; t++) {
      trajectory.at(t) = dynamics(trajectory.at(t - 1), actions.at(t - 1), t);
      actions.at(t).setZero();
      overall_cost += cost(trajectory.at(t), actions.at(t), t);
    }

    // Step 2: Calculate jacobian/hessian for x,u
    for (Time t = 0; t < T; t++) {
      j.at(t) = jacobian(trajectory.at(t), actions.at(t), t);

      g.at(t) = gradiant(trajectory.at(t), actions.at(t), t);
      h.at(t) = hessian(trajectory.at(t), actions.at(t), t, g.at(t));
    }
  }

  void backward_pass()
  {
    std::size_t num_iterations = 0;

    while (num_iterations < max_num_iterations) {
      // Step 3: Determine best control signal update
      Feedforwards k;
      Feedbacks K;

      Gradiant Q_dxu;  // Derivative of V(f(x, u))
      Hessian Q_ddxu;
      State v_dx = g.back().block(0, 0, X, 1);  // Derivative of V(x)
      Eigen::Matrix<double, X, X> v_ddx = h.back().block(0, 0, X, X);

      k.at(T - 1).setZero();
      K.at(T - 1).setZero();

      for (Time t = T - 2; t >= 0; t--) {
        // eq 4a
        Q_dxu.block(0, 0, X, 1) =
          g.at(t).block(0, 0, X, 1) +
          j.at(t).block(0, 0, X, X).transpose() *
          v_dx.block(0, 0, X, 1);
        // eq 4b
        Q_dxu.block(X, 0, U, 1) =
          g.at(t).block(X, 0, U, 1) +
          j.at(t).block(0, X, X, U).transpose() *
          v_dx.block(0, 0, X, 1);
        // eq 4c
        Q_ddxu.block(0, 0, X, X) =
          h.at(t).block(0, 0, X, X) +
          j.at(t).block(0, 0, X, X).transpose() *
          v_ddx.block(0, 0, X, X) *
          j.at(t).block(0, 0, X, X);
        // eq 4d
        Q_ddxu.block(X, 0, U, X) =
          h.at(t).block(X, 0, U, X) +
          j.at(t).block(0, X, X, U).transpose() *
          v_ddx.block(0, 0, X, X) *
          j.at(t).block(0, 0, X, X);
        // eq 4e
        Q_ddxu.block(X, X, U, U) =
          h.at(t).block(X, X, U, U) +
          j.at(t).block(0, X, X, U).transpose() *
          v_ddx.block(0, 0, X, X) *
          j.at(t).block(0, X, X, U);

        // eq 5b
        // Solve inverse with regulization to keep it from exploding
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, U, U>> eigensolver(Q_ddxu.block(
            X, X, U,
            U));
        if (eigensolver.info() != Eigen::Success) {
          std::cout << "Failed to solve eigen vectors" << std::endl;
          abort();
        }
        Eigen::Matrix<double, U, 1> eigen_vals = eigensolver.eigenvalues();
        Eigen::Matrix<double, U, U> eigen_vals_diag;
        eigen_vals_diag.setZero();
        for (int i = 0; i < U; i++) {
          if (eigen_vals[i] < 0) {
            eigen_vals[i] = 0;
          }
          eigen_vals[i] += alpha;
          eigen_vals_diag(i, i) = 1.0 / eigen_vals[i];
        }
        Eigen::Matrix<double, U, U> eigen_vecs = eigensolver.eigenvectors();
        Eigen::Matrix<double, U, U> inv = eigen_vecs * eigen_vals_diag * eigen_vecs.transpose();

        k.at(t) = -1 * inv * Q_dxu.block(X, 0, U, 1);
        K.at(t) = -1 * inv * Q_ddxu.block(X, 0, U, X);

        // eq 6b
        v_dx = Q_dxu.block(0, 0, X, 1) - K.at(t).transpose() * Q_ddxu.block(X, X, U, U) * k.at(t);
        // eq 6c
        v_ddx =
          Q_ddxu.block(0, 0, X, X) - K.at(t).transpose() * Q_ddxu.block(X, X, U, U) * K.at(t);
      }

      // Step 3.5: Update possible control signal, states, costs
      Cost test_cost;
      Trajectory test_trajectory;
      Actions test_actions;
      test_trajectory.at(0) = trajectory.at(0);
      test_actions.at(0) = actions.at(0) + k.at(0);  // xhat_0 == x_0 so Kt term goes away
      test_cost = cost(test_trajectory.at(0), test_actions.at(0), 0);
      for (Time t = 1; t < T; t++) {
        test_trajectory.at(t) = dynamics(
          test_trajectory.at(
            t - 1), test_actions.at(t - 1), t);
        test_actions.at(t) = actions.at(t) + k.at(t) + K.at(t) *
          (test_trajectory.at(t) - trajectory.at(t));
        test_cost = cost(test_trajectory.at(t), test_actions.at(t), t);
      }

      // Step 4: Compare Costs and update update size
      if (test_cost < overall_cost) {
        trajectory = test_trajectory;
        actions = test_actions;

        // If we converge, just return
        if (std::abs(test_cost - overall_cost) / test_cost < converge_threshold) {
          return;
        }

        overall_cost = test_cost;

        // Step 4.5: Calculate jacobian/hessian for x,u
        for (Time t = 0; t < T; t++) {
          j.at(t) = jacobian(trajectory.at(t), actions.at(t), t);

          g.at(t) = gradiant(trajectory.at(t), actions.at(t), t);
          h.at(t) = hessian(trajectory.at(t), actions.at(t), t, g.at(t));
        }

        alpha *= alpha_change;
      } else {
        alpha /= alpha_change;
      }

      num_iterations++;
    }
  }

  static constexpr std::size_t max_num_iterations = 1000;
  static constexpr double converge_threshold = 1e-6;
  static constexpr double alpha_change = 0.99;
  static constexpr double eps = 1e-3;

  double alpha = 1;  // backtracking serach parameter
  Trajectory trajectory;
  Actions actions;

  Jacobians j;
  Gradiants g;
  Hessians h;

  Cost overall_cost;
};

#endif  // TRAJECTORY_GENERATION__ILQR_PROBLEM_HPP_
