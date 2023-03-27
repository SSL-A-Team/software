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
    if (backward_pass()) {
      return trajectory;
    } else {
      return std::nullopt;
    }
  }

private:
  using StateJacobians = std::array<StateJacobian, T>;
  using CostHessians = std::array<CostHessian, T>;
  using CostGradiants = std::array<CostGradiant, T>;

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
    for (Time t = 0; t < T; t++) {
      j.at(t) = dynamics_jacobian(trajectory.at(t), actions.at(t), t);

      g.at(t) = cost_gradiant(trajectory.at(t), actions.at(t), t);
      h.at(t) = cost_hessian(trajectory.at(t), actions.at(t), t, g.at(t));
    }
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

  inline Eigen::Matrix<double, U, U> tikhonov_regularization(const Eigen::Matrix<double, U, U> & m)
  {
    // https://en.wikipedia.org/wiki/Ridge_regression#
    const Eigen::Matrix<double, U, U> & m_T = m.transpose();
    const Eigen::Matrix<double, U, U> & I = Eigen::Matrix<double, U, U>::Identity();
    return (m_T * m + lambda * I).inverse() * m_T;
  }

  bool backward_pass()
  {
    for (std::size_t num_ilqr_iterations = 0; num_ilqr_iterations < max_ilqr_iterations; num_ilqr_iterations++) {
      // Step 3: Determine best control signal update
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
        Eigen::Matrix<double, U, U> Q_uu_inv = tikhonov_regularization(Q_uu);

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

      // Step 3.5: Forward Pass
      // Eq 7a/b/c
      Cost candidate_cost;
      Trajectory candidate_trajectory;
      Actions candidate_actions;

      bool valid_forward_pass_found = false;
      for (std::size_t num_forward_pass_iterations = 0; num_forward_pass_iterations < max_forward_pass_iterations; num_forward_pass_iterations++) {
        // Generate likely forward pass
        double alpha = 1;

        candidate_trajectory.front() = trajectory.front();
        candidate_actions.front() = actions.front() + alpha * k.front();  // xhat_0 == x_0 so Kt term goes away
        candidate_trajectory.at(1) = dynamics(candidate_trajectory.at(0), candidate_actions.at(0), 0);
        candidate_cost = cost(candidate_trajectory.front(), candidate_actions.front(), 0);

        for (Time t = 1; t < T - 1; t++) {
          const State & x_hat_i = candidate_trajectory.at(t);
          const State & x_i = trajectory.at(t);
          const Input & u_i = actions.at(t);
          const Input & u_hat_i = u_i + alpha * k.at(t) + K.at(t) * (x_hat_i - x_i);
          candidate_actions.at(t) = u_hat_i;
          candidate_trajectory.at(t  + 1) = dynamics(x_hat_i, u_hat_i, t);
          candidate_cost += cost(candidate_trajectory.at(t), candidate_actions.at(t), t);
        }
        candidate_cost += cost(candidate_trajectory.back(), candidate_actions.back(), T);

        // From https://bjack205.github.io/papers/AL_iLQR_Tutorial.pdf
        // We can calculate the expected cost decrease
        double overall_candidate_cost_change = 0;
        for (const double dv : delta_v) {
          overall_candidate_cost_change += dv;
        }
        overall_candidate_cost_change *= alpha*alpha;
        double z = (overall_cost - candidate_cost) / -overall_candidate_cost_change;

        // Accept the solution if we are closeish to the actual expected decrease
        if (z > 1e-4 && z < 10) {
          valid_forward_pass_found = true;
          break;
        }

        // If it's not accepted, decrease the scale and retry
        alpha *= gamma;
      }

      // Step 4: Compare Costs and update update size
      if (valid_forward_pass_found && candidate_cost < overall_cost) {
        trajectory = candidate_trajectory;
        actions = candidate_actions;

        // If we converge, just return
        if (std::abs(candidate_cost - overall_cost) / candidate_cost < converge_threshold) {
          return true;
        }

        overall_cost = candidate_cost;

        // Step 4.5: Calculate jacobian/hessian for x,u
        for (Time t = 0; t < T; t++) {
          j.at(t) = dynamics_jacobian(trajectory.at(t), actions.at(t), t);

          g.at(t) = cost_gradiant(trajectory.at(t), actions.at(t), t);
          h.at(t) = cost_hessian(trajectory.at(t), actions.at(t), t, g.at(t));
        }
      } else {
        // Forward pass probably blew up
        lambda *= lambda_scale;
      }
    }

    return true;
  }

  static constexpr std::size_t max_ilqr_iterations = 1e3;
  static constexpr std::size_t max_forward_pass_iterations = 1e3;
  static constexpr double converge_threshold = 1e-6;
  static constexpr double eps = 1e-3;  // Auto differentiation epsilon

  static constexpr double gamma = 0.5;  // Backtracking scaling param
  static constexpr double lambda_scale = 2;  // When forward pass blows up, increase regularization

  double lambda = 1e-1;  // Ridge parameter for the Tikhonov regularization
  double alpha = 1e-3;  // Backtracking search parameter
  Trajectory trajectory;
  Actions actions;

  StateJacobians j;
  CostGradiants g;
  CostHessians h;

  Cost overall_cost;
};

#endif  // TRAJECTORY_GENERATION__ILQR_PROBLEM_HPP_
