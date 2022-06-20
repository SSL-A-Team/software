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
#include <iostream>
namespace TrapezoidalMotionProfile
{
Trajectory Generate3d(
  Eigen::Vector3d start, Eigen::Vector3d start_vel,
  Eigen::Vector3d end, Eigen::Vector3d end_vel,
  Eigen::Vector3d max_vel_limits,
  Eigen::Vector3d max_accel_limits,
  double dt)
{
  // Independently plan for each DOF
  std::array<Trajectory1d, 3> trajectories;

  // TODO(jneiger): Scale plans to longest DOF trajectory time length
  for (std::size_t i = 0; i < trajectories.size(); i++) {
    trajectories.at(i) = Generate1d(
      start(i), start_vel(i), end(i), end_vel(i), max_vel_limits(
        i), max_accel_limits(i), dt);
  }

  Trajectory output;

  std::size_t t_idx = 0;
  double t = 0;
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

Trajectory1d Generate1d(
  double start_pos, double start_vel, double end_pos, double end_vel,
  double max_vel, double max_accel, double dt)
{
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

  double phase_2_start_time;
  double phase_2_start_dist;
  double phase_2_start_vel;

  double phase_3_start_time;
  double phase_3_start_dist;
  double phase_3_start_vel;

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

  // Distance to cover
  double target_dist = end_pos - start_pos;

  // Time needed to accel/decel to/from max
  double t_start_to_max_vel = std::abs(max_vel - start_vel) / max_accel;
  double t_max_to_end_vel = std::abs(end_vel - max_vel) / max_accel;

  // Avg velocity over accel/decel
  double avg_vel_start_to_max_vel = (start_vel + max_vel) / 2;
  double avg_vel_max_to_end_vel = (max_vel + end_vel) / 2;

  // Distance covered during accel
  double d_covered_by_start_to_max_vel = avg_vel_start_to_max_vel * t_start_to_max_vel;
  double d_covered_by_max_to_end_vel = avg_vel_max_to_end_vel * t_max_to_end_vel;

  // Distanced covered by both accels
  double dist_covered_by_accels = d_covered_by_start_to_max_vel + d_covered_by_max_to_end_vel;

  // Skip phase 2 if that distance is too big
  // If it's too big, we need to shrink the max vel limit we accel/decel to/from
  bool skip_phase_two = dist_covered_by_accels > target_dist;

  if (!skip_phase_two) {
    // When phase 2 is included, stay at max speed for X time

    // How far we need phase 2 to cover in distance
    double d_covered_by_phase_2 = target_dist - dist_covered_by_accels;

    // Figure out how long that takes to cover that dist at max velocity
    double time_at_max_vel = d_covered_by_phase_2 / max_vel;

    phase_2_start_time = t_start_to_max_vel;
    phase_2_start_dist = d_covered_by_start_to_max_vel;
    phase_2_start_vel = max_vel;

    phase_3_start_time = t_start_to_max_vel + time_at_max_vel;
    phase_3_start_dist = phase_2_start_dist + d_covered_by_phase_2;
    phase_3_start_vel = max_vel;
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
    max_vel = sqrt((target_dist * max_accel + start_vel * start_vel + end_vel * end_vel) / 2);

    t_start_to_max_vel = (max_vel - start_vel) / max_accel;
    t_max_to_end_vel = (max_vel - end_vel) / max_accel;

    avg_vel_start_to_max_vel = (start_vel + max_vel) / 2;
    avg_vel_max_to_end_vel = (max_vel + end_vel) / 2;

    d_covered_by_start_to_max_vel = avg_vel_start_to_max_vel * t_start_to_max_vel;
    d_covered_by_max_to_end_vel = avg_vel_max_to_end_vel * t_max_to_end_vel;

    // Skip phase 2
    phase_2_start_time = t_start_to_max_vel;
    phase_2_start_dist = d_covered_by_start_to_max_vel;
    phase_2_start_vel = max_vel;

    phase_3_start_time = phase_2_start_time;
    phase_3_start_dist = phase_2_start_dist;
    phase_3_start_vel = phase_2_start_vel;
  }

  Trajectory1d output;

  double end_time = phase_3_start_time + t_max_to_end_vel;
  for (double t = 0.0; t <= end_time; t += dt) {
    Sample1d sample;
    sample.time = t;

    if (t < phase_2_start_time) {
      // Phase 1
      double d_into_phase_1 = start_pos;
      sample.pos = d_into_phase_1 + t * start_vel + 1.0 / 2.0 * max_accel * t * t;
      sample.vel = t * max_accel + start_vel;
      sample.accel = max_accel;
    } else if (t < phase_3_start_time) {
      // Phase 2
      double t_into_phase_2 = t - phase_2_start_time;
      double d_into_phase_2 = start_pos + phase_2_start_dist;
      sample.pos = d_into_phase_2 + max_vel * t_into_phase_2;
      sample.vel = phase_2_start_vel;
      sample.accel = 0;
    } else {
      // Phase 3
      double t_into_phase_3 = t - phase_3_start_time;
      double d_into_phase_3 = start_pos + phase_3_start_dist;
      sample.pos = d_into_phase_3 + t_into_phase_3 * max_vel - 1.0 / 2.0 * max_accel *
        t_into_phase_3 * t_into_phase_3;
      sample.vel = phase_3_start_vel - t_into_phase_3 * max_accel;
      sample.accel = -max_accel;
    }

    output.samples.push_back(sample);
  }

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
