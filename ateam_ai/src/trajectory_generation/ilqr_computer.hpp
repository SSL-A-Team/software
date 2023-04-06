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

#ifndef TRAJECTORY_GENERATION__ILQR_COMPUTER_HPP_
#define TRAJECTORY_GENERATION__ILQR_COMPUTER_HPP_

#include "trajectory_generation/ilqr_problem.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <optional>
#include <tuple>
#include <utility>

struct iLQRParams
{
  // Max number of outer loop iterations
  std::size_t max_ilqr_iterations = 1;
  // Max number of forward pass iterations
  std::size_t max_forward_pass_iterations = 1;
  // Convergence threshold for stopping (delta cost)
  double converge_threshold = 1e-1;
  // Backtracking scaling param
  double gamma = 0.5;
  // Minimum regularization value
  double lambda_min = 1e-6;
  // Initial regularization scale value
  double delta_zero = 2;
};

// Actually computes the calculations on a given iLQRProblem
// Based on https://homes.cs.washington.edu/~todorov/papers/TassaIROS12.pdf
template<int X, int U, int T>
class iLQRComputer
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

  using Trajectory = std::array<State, T>;
  using Actions = std::array<Input, T>;

  using Feedforwards = std::array<Input, T>;
  using Feedbacks = std::array<Eigen::Matrix<double, U, X>, T>;

  iLQRComputer(iLQRProblem<X, U, T> & problem, const iLQRParams & params)
  : problem(problem), params(params) {}

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
  using StateJacobians_x = std::array<StateJacobian_x, T>;
  using StateJacobians_u = std::array<StateJacobian_u, T>;
  using CostGradiants_x = std::array<CostGradiant_x, T>;
  using CostGradiants_u = std::array<CostGradiant_u, T>;
  using CostHessians_xx = std::array<CostHessian_xx, T>;
  using CostHessians_ux = std::array<CostHessian_ux, T>;
  using CostHessians_uu = std::array<CostHessian_uu, T>;

  void update_derivatives(const Trajectory & traj, const Actions & actions)
  {
    for (Time t = 0; t < T; t++) {
      std::tie(j_x.at(t), j_u.at(t)) = problem.dynamics_jacobian(traj.at(t), actions.at(t), t);

      std::tie(g_x.at(t), g_u.at(t)) = problem.cost_gradiant(traj.at(t), actions.at(t), t);
      std::tie(h_xx.at(t), h_ux.at(t), h_uu.at(t)) = problem.cost_hessian(
        traj.at(t), actions.at(
          t), t, g_x.at(t), g_u.at(t));
    }
  }

  Cost trajectory_action_cost(const Trajectory & traj, const Actions & actions)
  {
    Cost c = 0.0;
    for (Time t = 0; t < T; t++) {
      c += problem.cost(traj.at(t), actions.at(t), t);
    }
    return c;
  }

  void initial_rollout(const State & initial_state)
  {
    // Step 1: Forward Rollout
    trajectory.front() = initial_state;
    actions.front().setZero();
    overall_cost = problem.cost(trajectory.front(), actions.front(), 0);

    for (Time t = 1; t < T; t++) {
      trajectory.at(t) = problem.dynamics(trajectory.at(t - 1), actions.at(t - 1), t);
      actions.at(t).setZero();
      overall_cost += problem.cost(trajectory.at(t), actions.at(t), t);
    }

    // Step 2: Calculate jacobian/hessian for x,u
    update_derivatives(trajectory, actions);
  }

  std::tuple<Feedforwards, Feedbacks, std::array<std::pair<double, double>, T>> backward_pass()
  {
    Feedforwards k;
    Feedbacks K;
    std::array<std::pair<double, double>, T> delta_v;

    // Q-function - pseudo-Hamiltonian
    Eigen::Matrix<double, X, 1> Q_x;
    Eigen::Matrix<double, U, 1> Q_u;
    Eigen::Matrix<double, X, X> Q_xx;
    Eigen::Matrix<double, U, X> Q_ux;
    Eigen::Matrix<double, U, U> Q_uu;
    Eigen::Matrix<double, U, X> Q_ux_regu;
    Eigen::Matrix<double, U, U> Q_uu_regu;

    // Value at final time is the final cost
    CostGradiant_x V_x = g_x.back();
    CostHessian_xx V_xx = h_xx.back();

    k.at(T - 1).setZero();
    K.at(T - 1).setZero();

    for (Time t = T - 2; t >= 0; t--) {
      const CostGradiant_x & l_x = g_x.at(t);
      const CostGradiant_u & l_u = g_u.at(t);

      const StateJacobian_x & f_x = j_x.at(t);
      const StateJacobian_x_T & f_x_T = f_x.transpose();
      const StateJacobian_u & f_u = j_u.at(t);
      const StateJacobian_u_T & f_u_T = f_u.transpose();

      const CostHessian_xx & l_xx = h_xx.at(t);
      const CostHessian_ux & l_ux = h_ux.at(t);
      const CostHessian_uu & l_uu = h_uu.at(t);

      // eq 5a
      Q_x = l_x + f_x_T * V_x;
      // eq 5b
      Q_u = l_u + f_u_T * V_x;
      // eq 5c
      Q_xx = l_xx + f_x_T * V_xx * f_x;
      // eq 5d
      Q_ux = l_ux + f_u_T * V_xx * f_x;
      // eq 5e
      Q_uu = l_uu + f_u_T * V_xx * f_u;

      // eq 10a/10b
      Q_uu_regu = l_uu + f_u_T * (V_xx + lambda * Eigen::Matrix<double, X, X>::Identity()) * f_u;
      Q_ux_regu = l_ux + f_u_T * (V_xx + lambda * Eigen::Matrix<double, X, X>::Identity()) * f_x;

      // eq 10c/10d
      k.at(t) = -1 * Q_uu_regu.inverse() * Q_u;
      K.at(t) = -1 * Q_uu_regu.inverse() * Q_ux_regu;

      // eq 11b
      V_x = Q_x + K.at(t).transpose() * Q_uu * k.at(t) + K.at(t).transpose() * Q_u +
        Q_ux.transpose() * k.at(t);
      // eq 11c
      V_xx = Q_xx + K.at(t).transpose() * Q_uu * K.at(t) + K.at(t).transpose() * Q_ux +
        Q_ux.transpose() * K.at(t);
      // eq 11a
      // Split up into 2 halves, allows for simple computation of delta J(alpha)
      // when computing the line search
      // The first half can be scaled by alpha, the second half can be scaled by alpha^2
      delta_v.at(t) =
        std::make_pair(
        (k.at(t).transpose() * Q_u).value(),
        (0.5 * k.at(t).transpose() * Q_uu * k.at(t)).value());
    }

    return std::make_tuple(k, K, delta_v);
  }

  std::pair<Trajectory, Actions> ammend_trajectory_actions(
    const Feedforwards & k,
    const Feedbacks & K,
    const double alpha)
  {
    Trajectory candidate_trajectory;
    Actions candidate_actions;

    // eq 8a
    candidate_trajectory.front() = trajectory.front();
    for (Time t = 0; t < T - 1; t++) {
      const State & x_hat_i = candidate_trajectory.at(t);
      const State & x_i = trajectory.at(t);
      const Input & u_i = actions.at(t);
      // eq 12
      const Input & u_hat_i = u_i + alpha * k.at(t) + K.at(t) * (x_hat_i - x_i);
      candidate_actions.at(t) = u_hat_i;
      // eq 8c
      candidate_trajectory.at(t + 1) = problem.dynamics(x_hat_i, u_hat_i, t);
    }

    return std::make_pair(candidate_trajectory, candidate_actions);
  }

  std::optional<std::tuple<Trajectory, Actions, Cost>> do_backtracking_line_search(
    const Feedforwards & k,
    const Feedbacks & K,
    const std::array<std::pair<double, double>, T> & delta_v)
  {
    Cost candidate_cost;
    Trajectory candidate_trajectory;
    Actions candidate_actions;
    double alpha = 1;

    for (std::size_t num_forward_pass_iterations = 0;
      num_forward_pass_iterations < params.max_forward_pass_iterations;
      num_forward_pass_iterations++)
    {
      // Generate likely forward pass
      std::tie(candidate_trajectory, candidate_actions) = ammend_trajectory_actions(k, K, alpha);
      candidate_cost = trajectory_action_cost(candidate_trajectory, candidate_actions);

      // We can calculate the expected cost decrease
      double delta_j = 0;
      for (const auto & dv : delta_v) {
        // Note we can split up the cost into two parts, one with
        // a alpha coefficient, the second with an alpha^2 coefficient
        delta_j += alpha * dv.first + alpha * alpha * dv.second;
      }
      double z = (candidate_cost - overall_cost) / delta_j;

      // Accept the solution if we are closeish to the actual expected decrease
      if (z > 1e-4 && z < 10) {
        return std::make_tuple(candidate_trajectory, candidate_actions, candidate_cost);
      }

      // If it's not accepted, decrease the scale and retry
      alpha *= params.gamma;
    }

    return std::nullopt;
  }

  // Regulaization strategy from 2.F
  // https://homes.cs.washington.edu/~todorov/papers/TassaIROS12.pdf
  void decrease_regulation()
  {
    delta = std::min(1 / params.delta_zero, delta / params.delta_zero);
    if (lambda * delta > params.lambda_min) {
      lambda = lambda * delta;
    } else {
      lambda = 0;
    }
  }

  void increase_regulation()
  {
    delta = std::max(params.delta_zero, delta * params.delta_zero);
    lambda = std::max(params.lambda_min, lambda * delta);
  }

  bool converge()
  {
    for (std::size_t num_ilqr_iterations = 0; num_ilqr_iterations < params.max_ilqr_iterations;
      num_ilqr_iterations++)
    {
      // Step 3: Determine best control signal update
      Feedforwards k;
      Feedbacks K;
      std::array<std::pair<double, double>, T> delta_v;

      std::tie(k, K, delta_v) = backward_pass();

      // Step 3.5: Forward Pass
      auto maybe_traj_actions_costs = do_backtracking_line_search(k, K, delta_v);

      // Step 4: Compare Costs and update update size
      if (!maybe_traj_actions_costs.has_value()) {
        increase_regulation();
        continue;
      }

      Cost candidate_cost;
      Trajectory candidate_trajectory;
      Actions candidate_actions;
      std::tie(
        candidate_trajectory, candidate_actions,
        candidate_cost) = maybe_traj_actions_costs.value();

      if (candidate_cost < overall_cost) {
        trajectory = candidate_trajectory;
        actions = candidate_actions;

        // If we converge, just return
        if (std::abs(candidate_cost - overall_cost) < params.converge_threshold) {
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

  iLQRProblem<X, U, T> & problem;

  iLQRParams params;

  double delta = params.delta_zero;
  double lambda = 1e-1;  // Regularization

  Trajectory trajectory;
  Actions actions;

  StateJacobians_x j_x;
  StateJacobians_u j_u;
  CostGradiants_x g_x;
  CostGradiants_u g_u;
  CostHessians_xx h_xx;
  CostHessians_ux h_ux;
  CostHessians_uu h_uu;

  Cost overall_cost;
};

#endif  // TRAJECTORY_GENERATION__ILQR_COMPUTER_HPP_
