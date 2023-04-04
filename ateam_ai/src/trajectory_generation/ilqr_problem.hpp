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

  using Trajectory = std::array<State, T>;
  using Actions = std::array<Input, T>;

  using Feedforwards = std::array<Input, T>;
  using Feedbacks = std::array<Eigen::Matrix<double, U, X>, T>;

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

  std::optional<Trajectory> calculate(const State & initial_state)
  {
    initial_rollout(initial_state);
    if (converge()) {
      return trajectory;
    } else {
      return std::nullopt;
    }
  }

private:
  using StateJacobians = std::array<StateJacobian, T>;
  using CostHessians = std::array<CostHessian, T>;
  using CostGradiants = std::array<CostGradiant, T>;

  void update_derivatives(const Trajectory & traj, const Actions & actions)
  {
    for (Time t = 0; t < T; t++) {
      j.at(t) = dynamics_jacobian(traj.at(t), actions.at(t), t);

      g.at(t) = cost_gradiant(traj.at(t), actions.at(t), t);
      h.at(t) = cost_hessian(traj.at(t), actions.at(t), t, g.at(t));
    }
  }

  Cost trajectory_action_cost(const Trajectory & traj, const Actions & actions)
  {
    Cost c = 0.0;
    for (Time t = 0; t < T; t++) {
      c += cost(traj.at(t), actions.at(t), t);
    }
    return c;
  }

  void initial_rollout(const State & initial_state)
  {
    // Step 1: Forward Rollout
    trajectory.front() = initial_state;
    actions.front().setZero();
    overall_cost = cost(trajectory.front(), actions.front(), 0);

    for (Time t = 1; t < T; t++) {
      trajectory.at(t) = dynamics(trajectory.at(t - 1), actions.at(t - 1), t);
      actions.at(t).setZero();
      overall_cost += cost(trajectory.at(t), actions.at(t), t);
    }

    // Step 2: Calculate jacobian/hessian for x,u
    update_derivatives(trajectory, actions);
  }

  std::tuple<Feedforwards, Feedbacks, std::array<double, T>> backward_pass()
  {
    Feedforwards k;
    Feedbacks K;
    std::array<double, T> delta_v;

    // Q-function - pseudo-Hamiltonian
    Eigen::Matrix<double, X, 1> Q_x;
    Eigen::Matrix<double, U, 1> Q_u;
    Eigen::Matrix<double, X, X> Q_xx;
    Eigen::Matrix<double, U, X> Q_ux;
    Eigen::Matrix<double, U, U> Q_uu;

    // Value at final time is the final cost
    CostGradiant_x V_x = cost_gradiant_x(g.back());
    CostHessian_xx V_xx = cost_hessian_xx(h.back());

    k.at(T - 1).setZero();
    K.at(T - 1).setZero();

    for (Time t = T - 2; t >= 0; t--) {
      const CostGradiant_x & l_x = cost_gradiant_x(g.at(t));
      const CostGradiant_u & l_u = cost_gradiant_u(g.at(t));

      const StateJacobian_x & f_x = state_jacobian_x(j.at(t));
      const StateJacobian_x_T & f_x_T = f_x.transpose();
      const StateJacobian_u & f_u = state_jacobian_u(j.at(t));
      const StateJacobian_u_T & f_u_T = f_u.transpose();

      const CostHessian_xx & l_xx = cost_hessian_xx(h.at(t));
      const CostHessian_ux & l_ux = cost_hessian_ux(h.at(t));
      const CostHessian_uu & l_uu = cost_hessian_uu(h.at(t));

        // eq 4a
      Q_x = l_x + f_x_T * V_x;
        // eq 4b
      Q_u = l_u + f_u_T * V_x;
        // eq 4c
      Q_xx = l_xx + f_x_T * V_xx * f_x;
        // eq 4d
      Q_ux = l_ux + f_u_T * V_xx * f_x;
        // eq 4e
      Q_uu = l_uu + f_u_T * V_xx * f_u;

        // eq 5b
        // Solve inverse with regulization to keep it from exploding
      Eigen::Matrix<double, U, U> Q_uu_inv = inverse(Q_uu);

      k.at(t) = -1 * Q_uu_inv * Q_u;
      K.at(t) = -1 * Q_uu_inv * Q_ux;

      // eq 6a
      // Eigen doesn't like converting a 1x1 matrix to double :(
      delta_v.at(t) = -0.5 * (k.at(t).transpose() * Q_uu * k.at(t)).value();
      // eq 6b
      V_x = Q_x - K.at(t).transpose() * Q_uu * k.at(t);
      // eq 6c
      V_xx = Q_xx - K.at(t).transpose() * Q_uu * K.at(t);
    }

    return std::make_tuple(k, K, delta_v);
  }

  std::pair<Trajectory, Actions> ammend_trajectory_actions(
    const Trajectory & reference_traj,
    const Actions & reference_action,
    const Feedforwards & k,
    const Feedbacks & K,
    const double alpha)
  {
    // Eq 7a/b/c
    Trajectory candidate_trajectory;
    Actions candidate_actions;

    candidate_trajectory.front() = trajectory.front();
    for (Time t = 0; t < T - 1; t++) {
      const State & x_hat_i = candidate_trajectory.at(t);
      const State & x_i = trajectory.at(t);
      const Input & u_i = actions.at(t);
      const Input & u_hat_i = u_i + alpha * k.at(t) + K.at(t) * (x_hat_i - x_i);
      candidate_actions.at(t) = u_hat_i;
      candidate_trajectory.at(t  + 1) = dynamics(x_hat_i, u_hat_i, t);
    }

    return std::make_pair(candidate_trajectory, candidate_actions);
  }

  void decrease_regulation()
  {
    alpha *= alpha_change;
  }

  void increase_regulation()
  {
    alpha /= alpha_change;
  }

  inline StateJacobian_x state_jacobian_x(const StateJacobian & j)
  {
    return j.block(0, 0, X, X);
  }
  inline StateJacobian_u state_jacobian_u(const StateJacobian & j)
  {
    return j.block(0, X, X, U);
  }

  inline CostGradiant_x cost_gradiant_x(const CostGradiant & g)
  {
    return g.block(0, 0, X, 1);
  }
  inline CostGradiant_u cost_gradiant_u(const CostGradiant & g)
  {
    return g.block(X, 0, U, 1);
  }

  inline CostHessian_xx cost_hessian_xx(const CostHessian & h)
  {
    return h.block(0, 0, X, X);
  }
  inline CostHessian_ux cost_hessian_ux(const CostHessian & h)
  {
    return h.block(X, 0, U, X);
  }
  inline CostHessian_uu cost_hessian_uu(const CostHessian & h)
  {
    return h.block(X, X, U, U);
  }

  inline Eigen::Matrix<double, U, U> inverse(const Eigen::Matrix<double, U, U> & m)
  {
    // Solve inverse with regulization to keep it from exploding
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, U, U>> eigensolver(m);
    if (eigensolver.info() != Eigen::Success) {
      std::cout << "Failed to solve eigen vectors" << std::endl;
      // TODO(jneiger): Replace with tikhonov regularization + inverse which can't fail
      return m;
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
    return inv;
  }

  bool converge()
  {
    for (std::size_t num_ilqr_iterations = 0; num_ilqr_iterations < max_ilqr_iterations; num_ilqr_iterations++) {
      // Step 3: Determine best control signal update
      Feedforwards k;
      Feedbacks K;
      std::array<double, T> delta_v;

      std::cout << trajectory.back()(0) << std::endl;
      std::tie(k, K, delta_v) = backward_pass();

      // Step 3.5: Forward Pass
      Cost candidate_cost;
      Trajectory candidate_trajectory;
      Actions candidate_actions;
      std::tie(candidate_trajectory, candidate_actions) = ammend_trajectory_actions(trajectory, actions, k, K, 1.0);
      candidate_cost = trajectory_action_cost(candidate_trajectory, candidate_actions);


      if (candidate_cost < overall_cost) {
        trajectory = candidate_trajectory;
        actions = candidate_actions;

        // If we converge, just return
        if (std::abs(candidate_cost - overall_cost) / candidate_cost < converge_threshold) {
          return true;
        }

        overall_cost = candidate_cost;

        // Step 4.5: Calculate jacobian/hessian for x,u
        update_derivatives(trajectory, actions);

        decrease_regulation();
      } else {
        // Forward pass probably blew up
        increase_regulation();
      }
    }

    return true;
  }

  static constexpr std::size_t max_ilqr_iterations = 1;
  static constexpr std::size_t max_forward_pass_iterations = 1;
  static constexpr double alpha_change = 0.5;
  static constexpr double converge_threshold = 1e-1;
  static constexpr double eps = 1e-3;  // Auto differentiation epsilon

  double alpha = 1;
  Trajectory trajectory;
  Actions actions;

  StateJacobians j;
  CostGradiants g;
  CostHessians h;

  Cost overall_cost;
};

#endif  // TRAJECTORY_GENERATION__ILQR_PROBLEM_HPP_
