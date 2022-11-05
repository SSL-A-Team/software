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
  Trajectory empty;
  Sample3d sample;
  sample.time = world.current_time + plan_from_offset;
  sample.pose = current;
  sample.vel = current_vel;
  empty.samples.push_back(sample);
  if (!maybe_ball.has_value()) {
    return empty;
  }
  target.x() = maybe_ball.value().pos.x();
  target.y() = maybe_ball.value().pos.y();
  target.z() = 0;
  target_vel.x() = 0;
  target_vel.y() = 0;
  target_vel.z() = 0;
  Eigen::Vector3d max_vel{5, 5, 0.5};  // TODO(jneiger): Set as params
  Eigen::Vector3d max_accel{5, 5, 0.5};
  Trajectory t = TrapezoidalMotionProfile::Generate3d(
    current, current_vel, target,
    target_vel, max_vel, max_accel,
    kDt, 0);

  std::array<Eigen::Matrix<double, 6, 1>, kNumSamples> state;
  std::array<Eigen::Matrix<double, 3, 1>, kNumSamples> input;
  int i = 0;
  Eigen::Vector3d prev_vel;
  for (const auto& sample : t.samples) {
    if (i >= kNumSamples) {
      break;
    }
    state.at(i)(0) = sample.pose.x();
    state.at(i)(2) = sample.pose.y();
    state.at(i)(4) = sample.pose.z();
    state.at(i)(1) = sample.vel.x();
    state.at(i)(3) = sample.vel.y();
    state.at(i)(5) = sample.vel.z();
    
    input.at(i)(0) = sample.accel.x();
    input.at(i)(1) = sample.accel.y();
    input.at(i)(2) = sample.accel.z();
    prev_vel = sample.vel;
    i++;
    //std::cout << sample.pose.x() << " "  << sample.pose.y() << " : " <<sample.vel.x() << " "  <<sample.vel.y() << std::endl;
  }
  for (;i < kNumSamples; i++) {
    state.at(i) = state.at(i - 1);
    input.at(i) = 0*input.at(i - 1);
  }

  Eigen::Matrix<double, 6, 1> plan_from_state;
  plan_from_state <<
    plan_from.pos.x(),
    plan_from.vel.x(),
    plan_from.pos.y(),
    plan_from.vel.y(),
    plan_from.theta,
    plan_from.omega;
  MovingKickProblem problem;
  problem.actions = input;
  problem.target << target.x(), target_vel.x(),
                    target.y(), target_vel.y(),
                    target.z(), target_vel.z();
  auto maybe_trajectory = problem.calculate(plan_from_state);

  if (!maybe_trajectory.has_value()) {
    return empty;
  }

  Trajectory trajectory;
  double current_time = world.current_time + plan_from_offset;
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