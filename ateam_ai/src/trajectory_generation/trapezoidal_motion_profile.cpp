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

#include "trajectory_generation/trapezoidal_motion_profile.hpp"

#include <algorithm>
#include <numeric>
#include <iostream>

#include <ateam_common/angle.hpp>

namespace TrapezoidalMotionProfile
{
Trajectory Generate3d(
  const Eigen::Vector3d & start, const Eigen::Vector3d & start_vel,
  const Eigen::Vector3d & end, const Eigen::Vector3d & end_vel,
  const Eigen::Vector3d & max_vel_limits,
  const Eigen::Vector3d & max_accel_limits,
  const double dt, const double current_time)
{
  // Independently plan for each DOF
  std::array<Trajectory1d, 3> trajectories;

  // TODO(jneiger): Scale plans to longest DOF trajectory time length
  for (std::size_t i = 0; i < trajectories.size(); i++) {
    double s = start(i);
    double e = end(i);

    // If we're planning the angle, minimize the difference between the two by unwrapping
    if (i == 2) {
      double angle_diff = ateam_common::geometry::SignedSmallestAngleDifference(e, s);
      e = s + angle_diff;
    }

    trajectories.at(i) = Generate1d(
      start(i), start_vel(i), end(i), end_vel(i), max_vel_limits(
        i), max_accel_limits(i), dt);
  }

  Trajectory output;

  std::size_t t_idx = 0;
  double t = current_time;
  bool is_more_left = true;
  while (is_more_left) {
    is_more_left = false;
    Sample3d sample;
    sample.time = t;
    for (std::size_t i = 0; i < trajectories.size(); i++) {
      const auto & trajectory = trajectories.at(i);
      if (t_idx < trajectory.samples.size()) {
        sample.pose(i) = trajectory.samples.at(t_idx).pos;
        sample.vel(i) = trajectory.samples.at(t_idx).vel;
        sample.accel(i) = trajectory.samples.at(t_idx).accel;
        is_more_left = true;
      } else {
        sample.pose(i) = end(i);
        sample.vel(i) = end_vel(i);
        sample.accel(i) = 0;
      }
    }

    if (is_more_left) {
      output.samples.push_back(sample);
    }

    t += dt;
    t_idx++;
  }

  return output;
}

/**
 * Returns time (s) to accelerate from start vel to end vel given an acceleration limit
 */
double time_to_accelerate(const double start_vel, const double end_vel, const double accel_limit)
{
  return std::abs(end_vel - start_vel) / accel_limit;
}

/**
 * Return average velocity given the start vel and end vel, assuming a constant acceleration
 */
double avg_vel_over_acceleration(const double start_vel, const double end_vel)
{
  return std::midpoint(start_vel, end_vel);
}

/**
 * Returns distance (m) required to accelerate from start_vel to end_vel given an acceleration limit
 */
double dist_covered_during_acceleration(
  const double start_vel, const double end_vel,
  const double accel_limit)
{
  const double t = time_to_accelerate(start_vel, end_vel, accel_limit);
  const double avg_vel = avg_vel_over_acceleration(start_vel, end_vel);

  return t * avg_vel;
}

struct AccelerationPhaseInputs
{
  double start_vel;  // m/s
  double end_vel;  // m/s
  double accel_limit;  // abs(limiting_accel_value) == m/s^2
};

struct AccelerationPhaseResults
{
  double time_required;
  double distance_required;
};

AccelerationPhaseResults get_acceleration_phase_results(const AccelerationPhaseInputs inputs)
{
  AccelerationPhaseResults output;
  output.time_required = time_to_accelerate(inputs.start_vel, inputs.end_vel, inputs.accel_limit);
  output.distance_required = dist_covered_during_acceleration(
    inputs.start_vel, inputs.end_vel,
    inputs.accel_limit);

  return output;
}

struct PhaseCoeffs
{
  double start_time;
  double start_dist;
  double start_vel;
  double duration;
  double end_vel;
};

struct TrapezoidalCoeffs
{
  //    __________
  //   /          \.
  //  /            \.
  // /              \.
  // ^^^
  // Phase 1
  //    ^^^^^^^^^^
  //    Phase 2
  //              ^^^
  //              Phase 3
  //
  // Phase 1 - Accel to some "max" velocity
  // Phase 2 - Stay at some "max" velocity (optional)
  // Phase 3 - Decell to the end velocity
  PhaseCoeffs phase_1;
  PhaseCoeffs phase_2;
  PhaseCoeffs phase_3;
};

Trajectory1d sample_trapezoidal_profile(
  const TrapezoidalCoeffs coeffs, const double accel_limit,
  const double dt)
{
  const auto position = [](const double t, const double v_initial, const double a) {
      return v_initial * t + 1. / 2. * a * std::pow(t, 2);
    };
  const auto velocity = [](const double t, const double v_initial, const double a) {
      return v_initial + a * t;
    };
  const auto sign = [](const double x) {
    return x / abs(x);
  };

  Trajectory1d output;

  const double end_time = coeffs.phase_3.start_time + coeffs.phase_3.duration;
  for (double t = 0.0; t <= end_time; t += dt) {
    Sample1d sample;
    sample.time = t;

    if (t < coeffs.phase_2.start_time) {
      // Phase 1
      double accel_direction = sign(coeffs.phase_1.end_vel - coeffs.phase_1.start_vel);
      sample.pos = position(t, coeffs.phase_1.start_vel, accel_direction * accel_limit);
      sample.vel = velocity(t, coeffs.phase_1.start_vel, accel_direction * accel_limit);
      sample.accel = accel_limit;
    } else if (t < coeffs.phase_3.start_time) {
      // Phase 2
      double t_into_phase_2 = t - coeffs.phase_2.start_time;
      double d_into_phase_2 = coeffs.phase_2.start_dist;
      sample.pos = d_into_phase_2 + position(t_into_phase_2, coeffs.phase_2.start_vel, 0);
      sample.vel = velocity(t_into_phase_2, coeffs.phase_2.start_vel, 0);
      sample.accel = 0;
    } else {
      // Phase 3
      double t_into_phase_3 = t - coeffs.phase_3.start_time;
      double d_into_phase_3 = coeffs.phase_3.start_dist;
      double accel_direction = sign(coeffs.phase_3.end_vel - coeffs.phase_3.start_vel);
      sample.pos = d_into_phase_3 +
        position(t_into_phase_3, coeffs.phase_3.start_vel, accel_direction * accel_limit);
      sample.vel = velocity(t_into_phase_3, coeffs.phase_3.start_vel, accel_direction * accel_limit);
      sample.accel = -accel_limit;
    }

    output.samples.push_back(sample);
  }

  return output;
}

Trajectory1d Generate1d(
  double start_pos, double start_vel, double end_pos, double end_vel,
  const double max_vel, const double max_accel, const double dt)
{
  // Catch for us already at the target causing the distance to travel
  // being so small that it creates nan values
  if (std::abs(start_pos - end_pos) < 1e-6 && std::abs(start_vel - end_vel) < 1e-6) {
    Trajectory1d traj;
    traj.samples.push_back(Sample1d{.time=0, .pos=end_pos, .vel=end_vel, .accel=0});
    return traj;
  }

  // Invert so the start is always before the end
  // and we only have to deal with positive values
  bool invert = start_pos > end_pos;
  if (invert) {
    start_pos *= -1;
    start_vel *= -1;
    end_pos *= -1;
    end_vel *= -1;
  }

  // Note, distances are distance from start_pos internally
  // The output will be shifted back into world coordinates
  TrapezoidalCoeffs trapezoidal_coeffs;
  trapezoidal_coeffs.phase_1.start_time = 0;
  trapezoidal_coeffs.phase_1.start_dist = 0;
  trapezoidal_coeffs.phase_1.start_vel = start_vel;

  // Phase 1 is start_vel to max_vel
  const AccelerationPhaseInputs phase_1_to_vel_limit_input{
    .start_vel = start_vel,
    .end_vel = max_vel,
    .accel_limit = max_accel};

  // Phase 3 is max_accel to end_vel
  const AccelerationPhaseInputs phase_3_from_vel_limit_input{
    .start_vel = max_vel,
    .end_vel = end_vel,
    .accel_limit = max_accel};

  const AccelerationPhaseResults phase_1_to_vel_limit = get_acceleration_phase_results(
    phase_1_to_vel_limit_input);
  const AccelerationPhaseResults phase_3_from_vel_limit = get_acceleration_phase_results(
    phase_3_from_vel_limit_input);

  // Distanced covered by both accels
  // Skip phase 2 if that distance is too big
  // If it's too big, we need to shrink the max vel limit we accel/decel to/from
  const double target_dist = end_pos - start_pos;  // Distance to cover
  const double dist_covered_by_accels = phase_1_to_vel_limit.distance_required +
    phase_3_from_vel_limit.distance_required;
  const bool skip_phase_two = dist_covered_by_accels > target_dist;

  if (!skip_phase_two) {
    // When phase 2 is included, stay at max speed for X time

    // How far we need phase 2 to cover in distance
    double d_covered_by_phase_2 = target_dist - dist_covered_by_accels;

    // Figure out how long that takes to cover that dist at max velocity
    double time_at_max_vel = d_covered_by_phase_2 / max_vel;

    trapezoidal_coeffs.phase_1.duration = phase_1_to_vel_limit.time_required;
    trapezoidal_coeffs.phase_1.end_vel = max_vel;

    trapezoidal_coeffs.phase_2.start_time = phase_1_to_vel_limit.time_required;
    trapezoidal_coeffs.phase_2.start_dist = phase_1_to_vel_limit.distance_required;
    trapezoidal_coeffs.phase_2.start_vel = max_vel;
    trapezoidal_coeffs.phase_2.duration = time_at_max_vel;
    trapezoidal_coeffs.phase_2.end_vel = max_vel;

    trapezoidal_coeffs.phase_3.start_time = trapezoidal_coeffs.phase_2.start_time + time_at_max_vel;
    trapezoidal_coeffs.phase_3.start_dist = trapezoidal_coeffs.phase_2.start_dist +
      d_covered_by_phase_2;
    trapezoidal_coeffs.phase_3.start_vel = max_vel;
    trapezoidal_coeffs.phase_3.duration = phase_3_from_vel_limit.time_required;
    trapezoidal_coeffs.phase_3.end_vel = end_vel;
  } else {
    // Figure out the max vel needed to get the target dist
    // Solve the above equations for max_velocity
    // https://www.wolframalpha.com/input?i=solve+d+%3D+m*+%28m-+s%29+%2F+a%2B+s+*+%28m-+s%29+%2F+a%2B+m*+%28m-+e%29+%2F+a+%2B+e*+%28m-+e%29+%2F+a+for+m

    // max = max vel
    // svel = start vel
    // evel = end vel
    // d = target_distance

    // ts = (max - svel) / max_accel
    // te = (max - evel) / max_accel

    // avgs = (max + svel) * ts
    // avge = (max + evel) * te

    // d = avgs + avge
    // d = (max + svel) * ts + (max + evel) * te
    // d = max * ts + svel * ts + max * te + evel * te
    // d = max * (max - svel) / max_accel + svel * (max - svel) / max_accel +
    //     max * (max - evel) / max_accel + evel * (max - evel) / max_accel
    // d * max_accel = max * (max - svel) + svel * (max - svel) +
    //                 max * (max - evel) + evel * (max - evel)
    // d * max_accel = max^2 - max * svel + svel * max - svel^2 +
    //                 max^2 - max * evel + evel * max - evel^2
    // d * max_accel + svel^2 + evel^2 = 2 * max^2
    // (d * max_accel + svel^2 + evel^2) / 2 = max^2
    // sqrt((d * max_accel + svel^2 + evel^2) / 2) = +-max

    // Don't allow negative speed so we can just assume positive is only correct value
    double nonlimit_max_vel =
      sqrt((target_dist * max_accel + start_vel * start_vel + end_vel * end_vel) / 2);

    // Phase 1 is start_vel to nonlimit_max_vel
    const AccelerationPhaseInputs phase_1_to_nonlimit_vel_input{
      .start_vel = start_vel,
      .end_vel = nonlimit_max_vel,
      .accel_limit = max_accel};

    // Phase 3 is max_accel to end_vel
    const AccelerationPhaseInputs phase_3_from_nonlimit_vel_input{
      .start_vel = nonlimit_max_vel,
      .end_vel = end_vel,
      .accel_limit = max_accel};

    const AccelerationPhaseResults phase_1_to_nonlimit_vel = get_acceleration_phase_results(
      phase_1_to_nonlimit_vel_input);
    const AccelerationPhaseResults phase_3_from_nonlimit_vel = get_acceleration_phase_results(
      phase_3_from_nonlimit_vel_input);

    trapezoidal_coeffs.phase_1.duration = phase_1_to_nonlimit_vel.time_required;
    trapezoidal_coeffs.phase_1.end_vel = nonlimit_max_vel;

    // Skip phase 2
    trapezoidal_coeffs.phase_2.start_time = phase_1_to_nonlimit_vel.time_required;
    trapezoidal_coeffs.phase_2.start_dist = phase_1_to_nonlimit_vel.distance_required;
    trapezoidal_coeffs.phase_2.start_vel = nonlimit_max_vel;
    trapezoidal_coeffs.phase_2.duration = 0;
    trapezoidal_coeffs.phase_2.end_vel = nonlimit_max_vel;

    trapezoidal_coeffs.phase_3.start_time = trapezoidal_coeffs.phase_2.start_time;
    trapezoidal_coeffs.phase_3.start_dist = trapezoidal_coeffs.phase_2.start_dist;
    trapezoidal_coeffs.phase_3.start_vel = nonlimit_max_vel;
    trapezoidal_coeffs.phase_3.duration = phase_3_from_nonlimit_vel.time_required;
    trapezoidal_coeffs.phase_3.end_vel = end_vel;
  }

  Trajectory1d output = sample_trapezoidal_profile(trapezoidal_coeffs, max_accel, dt);

  // Add offset start pos
  std::for_each(
    output.samples.begin(),
    output.samples.end(),
    [start_pos](Sample1d & sample) {
      sample.pos += start_pos;
    });

  if (invert) {
    for (auto & sample : output.samples) {
      sample.pos *= -1;
      sample.vel *= -1;
      sample.accel *= -1;
    }
  }

  return output;
}
}  // namespace TrapezoidalMotionProfile
