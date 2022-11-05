#include "trajectory_generation/moving_kick_planner.hpp"

#include "trajectory_generation/ilqr_problem.hpp"
#include "trajectory_generation/trapezoidal_motion_profile.hpp"
#include <iostream>
namespace moving_kick_planner {

constexpr double kDt = 0.1;
constexpr double kTimeHorizon = 3;  // s
constexpr int kNumSamples = kTimeHorizon * 1 / kDt;

// X {x, xvel, y, yvel, theta, omega}
class MovingKickProblem : public iLQRProblem<6, 3, kNumSamples>
{
public:
  Eigen::Matrix<double, 6, 1> target;

  State dynamics(const State & x_t1, const Input & u_t1, Time t) override
  {
    Eigen::Matrix<double, 6, 6> A;
    A << 0, 1, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0,
         0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 1,
         0, 0, 0, 0, 0, 0;
    Eigen::Matrix<double, 6, 3> B;
    B << 0, 0, 0,
         1, 0, 0,
         0, 0, 0,
         0, 1, 0,
         0, 0, 0,
         0, 0, 1;
    
    return x_t1 + kDt * (A * x_t1 + B * u_t1);
  }

  // virtual Jacobian jacobian(const State & x, const Input & u, Time t) override
  // {
  //   Jacobian out;
  //   out << 0, 1, 0, 0, 0, 0, 0, 0, 0,
  //          0, 0, 0, 0, 0, 0, 1, 0, 0,
  //          0, 0, 0, 1, 0, 0, 0, 0, 0,
  //          0, 0, 0, 0, 0, 0, 0, 1, 0,
  //          0, 0, 0, 0, 0, 1, 0, 0, 0,
  //          0, 0, 0, 0, 0, 0, 0, 0, 1;
  //   return out;
  // }

  Cost cost(const State & x, const Input & u, Time t) override
  {
    constexpr double final_gain = 1e6;
    if (t == kNumSamples - 1) {
      return final_gain * (target - x).dot(target - x);
    } else {
      return kDt * u.dot(u);
    }
  }

  // Gradiant gradiant(const State & x, const Input & u, Time t) override
  // {
  //   constexpr double final_gain = 1e6;
  //   Gradiant grad;
  //   grad << 0,0,0,0,0,0,0,0,0;
  //   if (t == kNumSamples - 1) {
  //     grad.block(0, 0, 6, 1) = -2 * final_gain * (target - x);
  //     return grad;
  //   } else {
  //     grad.block(6, 0, 3, 1) = 2 * kDt * u;
  //     return grad;
  //   }
  // }

  // virtual Hessian hessian(const State & x, const Input & u, Time t, const Gradiant & g)
  // {
  //   constexpr double final_gain = 1e6;
  //   if (t == kNumSamples - 1) {
  //   Hessian out;
  //     out << 2 * final_gain, 0, 0, 0, 0, 0, 0, 0, 0,
  //            0, 2 * final_gain, 0, 0, 0, 0, 0, 0, 0,
  //            0, 0, 2 * final_gain, 0, 0, 0, 0, 0, 0,
  //            0, 0, 0, 2 * final_gain, 0, 0, 0, 0, 0,
  //            0, 0, 0, 0, 2 * final_gain, 0, 0, 0, 0,
  //            0, 0, 0, 0, 0, 2 * final_gain, 0, 0, 0,
  //            0, 0, 0, 0, 0, 0, 0, 0, 0,
  //            0, 0, 0, 0, 0, 0, 0, 0, 0,
  //            0, 0, 0, 0, 0, 0, 0, 0, 0;
  //     return out;
  //   } else {
  //   Hessian out;
  //     out << 0, 0, 0, 0, 0, 0, 0, 0, 0,
  //            0, 0, 0, 0, 0, 0, 0, 0, 0,
  //            0, 0, 0, 0, 0, 0, 0, 0, 0,
  //            0, 0, 0, 0, 0, 0, 0, 0, 0,
  //            0, 0, 0, 0, 0, 0, 0, 0, 0,
  //            0, 0, 0, 0, 0, 0, 0, 0, 0,
  //            0, 0, 0, 0, 0, 0, 2 * kDt, 0, 0,
  //            0, 0, 0, 0, 0, 0, 0, 2 * kDt, 0,
  //            0, 0, 0, 0, 0, 0, 0, 0, 2 * kDt;
  //     return out;
  //   }
  // }
};

Trajectory PlanMovingKick(
  const Robot & plan_from,
  const double plan_from_offset,
  const World & world)
{
  Eigen::Vector3d current, current_vel, target, target_vel;
  current.x() = plan_from.pos.x();
  current.y() = plan_from.pos.y();
  current.z() = 0;  // current_robot.theta;
  current_vel.x() = plan_from.vel.x();
  current_vel.y() = plan_from.vel.y();
  current_vel.z() = 0;  // current_robot.omega;

  auto maybe_ball = world.get_unique_ball();
  if (!maybe_ball.has_value()) {
    return Trajectory();
  }
  target.x() = maybe_ball.value().pos.x();
  target.y() = maybe_ball.value().pos.y();
  target.z() = 0;
  target_vel.x() = 0;
  target_vel.y() = 0;
  target_vel.z() = 0;
  Eigen::Vector3d max_vel{2, 2, 0.5};  // TODO(jneiger): Set as params
  Eigen::Vector3d max_accel{2, 2, 0.5};
  Trajectory t = TrapezoidalMotionProfile::Generate3d(
    current, current_vel, target,
    target_vel, max_vel, max_accel,
    kDt, 0);

  std::array<Eigen::Matrix<double, 6, 1>, kNumSamples> state;
  std::array<Eigen::Matrix<double, 3, 1>, kNumSamples> input;
  int i = 0;
  for (const auto& sample : t.samples) {
    if (i >= kNumSamples) {
      break;
    }
    state.at(i)(0) = current.x();
    state.at(i)(2) = current.y();
    state.at(i)(4) = current.z();
    state.at(i)(1) = current_vel.x();
    state.at(i)(3) = current_vel.y();
    state.at(i)(5) = current_vel.z();
    
    input.at(i)(0) = current_vel.x();
    input.at(i)(1) = current_vel.y();
    input.at(i)(2) = current_vel.z();
    i++;
  }

  Eigen::Matrix<double, 6, 1> plan_from_state;
  plan_from_state <<
    world.our_robots.at(1).value().pos.x(),
    world.our_robots.at(1).value().vel.x(),
    world.our_robots.at(1).value().pos.y(),
    world.our_robots.at(1).value().vel.y(),
    plan_from.theta,
    plan_from.omega;
  MovingKickProblem problem;
  problem.actions = input;
  problem.target << maybe_ball.value().pos.x(), maybe_ball.value().pos.y(), 0, 0, 0, 0;
  auto maybe_trajectory = problem.calculate(plan_from_state);

  if (!maybe_trajectory.has_value()) {
    return Trajectory();
  }

  Trajectory trajectory;
  double current_time = world.current_time;
  Eigen::Vector3d prevvel{0,0,0};
  double mv = 0;
  for (const auto & state : maybe_trajectory.value()) {
    Sample3d sample;
    sample.time = current_time;
    sample.pose = Eigen::Vector3d{state(0), state(2), state(4)};
    sample.vel = Eigen::Vector3d{state(1), state(3), state(5)};
    mv = std::max(sample.vel.norm(), mv);
    sample.accel = (sample.vel - prevvel) / kDt;
    prevvel = sample.vel;
    trajectory.samples.push_back(sample);
    current_time += kDt;
  }

  return trajectory;
}

};