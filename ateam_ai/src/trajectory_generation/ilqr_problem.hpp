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

// Defines the abstract definition and solution to the iLQR problem
// https://homes.cs.washington.edu/~todorov/papers/TassaICRA14.pdf
// https://studywolf.wordpress.com/2016/02/03/the-iterative-linear-quadratic-regulator-method/
// X = num states, U = num inputs, T = num timesteps
template<typename X, typename U, typename T>
class iLQRProblem {
public:
  using State = Eigen::Vector<X>;
  using Input = Eigen::Vector<U>;
  using Jacobian = Eigen::Matrix<X, X + U>;
  using Cost = double;
  using Gradiant = Eigen::Vector<X + U>;
  using Hessian = Eigen::Matrix<X + U, X + U>;
  using Time = int;

  using Trajectory = std::array<State, T>;
  using Actions = std::array<Input, T>;

  using Feedforwards = std::array<Input, T>;
  using Feedbacks = std::array<Eigen::Matrix<U, X>, T>;

  // Assume second derivative of dynamics is 0 so we don't have do tensor math
  virtual State dynamics(const State& x_t1, const Input& u_t1, Time t) = 0;
  virtual Jacobian jacobian(const State& x, const Input& u, Time t) {
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

  virtual Cost cost(const State& x, const Input& u, Time t) = 0;
  virtual Gradiant gradiant(const State& x, const Input& u, Time t) {
    Gradiant out;

    Cost c = cost(x, u, t);
    for (std::size_t i = 0; i < X; i++) {
      State x_eps = x;
      x_eps(i) += eps;
      out(i) = (cost(x_eps, u, t) - c) / eps;
    }

    for (std::size_t i = 0; i < U; i++) {
      Input u_eps = u;
      u_eps(i) += eps;
      out(i + X) = (cost(x, u_eps, t) - c) / eps;
    }

    return out;
  }

  virtual Hessian hessian(const State& x, const Input& u, Time t, const Gradiant& g) {
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

  Trajectory calculate(const State& initial_state) {
    forward_rollout(initial_state);
    backward_pass();

    return trajectory;
  }

private:
  using Jacobians = std::array<Jacobian, T>;
  using Hessians = std::array<Hessian, T>;
  using Gradiants = std::array<Gradiant, T>;


  void forward_rollout(const State& initial_state) {
    // Step 1: Forward Rollout
    trajectory.front() = initial_state;
    actions.front() = Input::Zeros();
    overall_cost = cost(trajectory.at(0), actions.at(0), 0);

    for (Time t = 1; t < T; t++) {
      trajectory.at(t) = trajectory.at(t - 1) + dynamics(trajectory.at(t - 1), actions.at(i - 1));
      actions.at(t) = Input::Zeros();
      overall_cost += cost(trajectory.at(t), actions.at(t), t);
    }

    // Step 2: Calculate jacobian/hessian for x,u
    for (Time t = 0; t < T; i++) {
      j.at(t) = jacobian(trajectory.at(t), actions.at(t), t);

      g.at(t) = gradiant(trajectory.at(t), actions.at(t), t);
      h.at(t) = hessian(trajectory.at(t), actions.at(t), t, j.at(t));
    }
  }

  void backward_pass() {
    std::size_t num_iterations = 0;

    while (num_iterations < max_num_iterations) {
      // Step 3: Determine best control signal update
      // Determine 
      Feedforwards k;
      Feedbacks K;

      Cost cost_at_next_time;
      Gradiant q_dxu; // Derivative of V(f(x, u))
      Hessian q_ddxu;
      Gradiant v_dx = g.back(); // Derivative of V(x)
      Hessian v_ddx = h.back();

      k.at(T - 1) = k.at(T - 1)::Zeros();
      K.at(T - 1) = K.at(T - 1)::Zeros();

      for (Time t = T - 2; t >= 0; t--) {
        // eq 4a
        Q_dxu.block<X, 1>(0, 0) =
          g.at(t).block<X, 1>(0, 0) +
          j.at(t).block<X, X>(0, 0).transpose() *
          v_dx.block<X, 1>(0, 0);
        // eq 4b
        Q_dxu.block<U, 1>(X, 0) =
          g.at(t).block<U, 1>(X, 0) +
          j.at(t).block<X, U>(0, X).transpose() *
          v_dx.block<X, 1>(X, 0);
        // eq 4c
        Q_ddxu.block<X, X>(0, 0) =
          h.at(t).block<X, X>(0, 0) +
          j.at(t).block<X, X>(0, 0).transpose() *
          v_ddx.block<X, X>(0, 0) *
          j.at(t).block<X, X>(0, 0);
        // eq 4d
        Q_ddxu.block<U, X>(X, 0) =
          h.at(t).block<U, X>(X, 0) +
          j.at(t).block<X, U>(0, X).transpose() *
          v_ddx.block<X, X>(0, 0) *
          j.at(t).block<X, U>(0, X);
        // eq 4e
        Q_ddxu.block<U, U>(X, X) =
          g.at(t).block<U, U>(X, X) +
          j.at(t).block<X, U>(0, X).transpose() *
          v_ddx.block<X, X>(0, 0) *
          j.at(t).block<X, U>(0, X);

        // eq 5b
        k.at(t) = -Q_ddxu.block<U, U>(X, X).inverse() * Q_dxu.block<U, 1>(X, 0);
        K.at(t) = -Q_ddxu.block<U, U>(X, X).inverse() * Q_ddxu.block<U, X>(X, 0);

        // eq 6b
        v_dx = Q_dxu.block<X, 1>(0, 0) - K.at(t).transpose() * Q_ddxu.block<U, U>(X, X) * k.at(t);
        // eq 6c
        v_ddx = Q_ddxu.block<X, X>(0, 0) - K.at(t).transepose() * Q_ddxu.block<U, U>(X, X) * K.at(t);
      }

      // Step 3.5: Update possible control signal, states, costs
      Cost test_cost;
      Trajectory test_trajectory;
      Actions test_actions;
      test_trajectory.at(0) = trajectory.at(0);
      test_actions.at(0) = actions.at(0) + k.at(t);  // xhat_0 == x_0 so Kt term goes away
      test_cost = cost(test_trajectory.at(0), test_actions.at(0), 0);
      for (Time t = 1; t < T; t++) {
        test_trajectory.at(t) = test_trajectory.at(t - 1) + dynamics(test_trajectory.at(t - 1), test_actions.at(t - 1));
        test_actions.at(t) = actions.at(t) + alpha * k.at(t) + K.at(t) * (test_trajectory.at(t) - trajectory.at(t));
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
        for (Time t = 0; t < T; i++) {
          j.at(t) = jacobian(trajectory.at(t), actions.at(t), t);

          g.at(t) = gradiant(trajectory.at(t), actions.at(t), t);
          h.at(t) = hessian(trajectory.at(t), actions.at(t), t, j.at(t));
        }

        alpha *= alpha_change;
      } else {
        alpha /= alpha_change;
      }

      num_iterations++;
    }
  }

  constexpr std::size_t max_num_iterations = 100;
  constexpr double converge_threshold = 0.1;
  constexpr double alpha_change = 0.9;
  constexpr double eps = 1e-5;

  double alpha = 1;  // backtracking serach parameter
  Trajectory trajectory;
  Actions actions;

  Jacobians j;
  Gradiants g;
  Hessians h;

  Cost overall_cost;
};

#endif  // TRAJECTORY_GENERATION__ILQR_PROBLEM_HPP_
