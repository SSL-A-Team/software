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

#include "trajectory_generation/b_spline_wrapper.hpp"

#include <angles/angles.h>

#include <algorithm>

#include <ateam_common/status.hpp>

#include "trajectory_generation/b_spline.hpp"
#include "trajectory_generation/trapezoidal_motion_profile.hpp"

namespace BSplineWrapper
{

Trajectory Generate(
  const std::vector<Eigen::Vector2d> waypoints,
  const double start_heading,
  const double end_heading,
  const Eigen::Vector3d & start_vel,
  const Eigen::Vector3d & end_vel,
  const Eigen::Vector3d & max_vel_limits,
  const Eigen::Vector3d & max_accel_limits,
  const double dt, const double current_time)
{
  ATEAM_CHECK(waypoints.size() >= 2, "Must have at least 2 waypoints");

  //
  // Generate the base "trajectories"
  //

  // TODO(jneiger): Split x/y limits to be independent
  BSpline::Input b_input;
  b_input.data_points = waypoints;
  b_input.initial_vel = Eigen::Vector2d{start_vel.x(), start_vel.y()};
  b_input.end_vel = Eigen::Vector2d{end_vel.x(), end_vel.y()};
  b_input.max_accel = max_accel_limits.x();  // Assume X/Y limits are the same
  b_input.max_vel = max_vel_limits.x();

  // Need enough samples that the conversion from curve to a series of line segments
  // is still close enough to reality for the trajectory following.
  const std::size_t kNumSamples = 40;
  // Plan XY trajectory (output is function of position along curve)
  BSpline::Output b_output = BSpline::build_and_sample_spline(b_input, kNumSamples);

  // Plan heading trajectory (output is function of time)
  double modified_end_heading = start_heading + angles::shortest_angular_distance(
    start_heading,
    end_heading);

  std::array<TrapezoidalMotionProfile::Trajectory1d, 3> trajectories;
  trajectories.at(2) =
    TrapezoidalMotionProfile::Generate1d(
    start_heading,
    start_vel.z(),
    modified_end_heading,
    end_vel.z(),
    max_vel_limits.z(),
    max_accel_limits.z(), dt);

  //
  // Sample b spline output to be on the same time series as the heading
  //

  // Step through the output and figure out given the speed / accel, the time that segment
  // If the time since the start matches the target sample time, interpolate that segment
  double prev_segment_t = 0.0;
  double target_t = 0.0;
  for (int i = 0; i < b_output.samples.size() - 1; i++) {
    Eigen::Vector2d diff = b_output.samples.at(i + 1).p - b_output.samples.at(i).p;
    double segment_dist = (b_output.samples.at(i + 1).p - b_output.samples.at(i).p).norm();
    double average_v = (b_output.samples.at(i + 1).v - b_output.samples.at(i).v) / 2;

    // This check is probably overly restrictive
    // Really just want to include acceleration in the t calc
    // with some upper limit so we don't break on bad bspline segment accels at 0 vel samples
    // ATEAM_CHECK(average_v == 0, "Average velocity must be non-zero across a segment");
    double end_of_segment_t = prev_segment_t + std::min(segment_dist / std::abs(average_v), 1.0);

    // target_t is in this segment
    if (target_t >= prev_segment_t && target_t < end_of_segment_t) {
      // Interpolate the actual position / velocity / accel
      double coeff = (target_t - prev_segment_t) / (end_of_segment_t - prev_segment_t);
      double vel_mag = b_output.samples.at(i).v + (target_t - prev_segment_t) * b_output.samples.at(
        i).a;

      TrapezoidalMotionProfile::Sample1d x_sample{
        .time = target_t,
        .pos = (b_output.samples.at(i).p + coeff * diff).x(),
        .vel = (vel_mag * diff.normalized()).x(),
        .accel = (b_output.samples.at(i).a * diff.normalized()).x()
      };
      TrapezoidalMotionProfile::Sample1d y_sample{
        .time = target_t,
        .pos = (b_output.samples.at(i).p + coeff * diff).y(),
        .vel = (vel_mag * diff.normalized()).y(),
        .accel = (b_output.samples.at(i).a * diff.normalized()).y()
      };

      trajectories.at(0).samples.push_back(x_sample);
      trajectories.at(1).samples.push_back(y_sample);
      target_t += dt;
    }

    if (target_t >= end_of_segment_t) {
      prev_segment_t = end_of_segment_t;
    }
  }

  //
  // Combine the trajectories
  //
  Trajectory output;

  // TODO(jneiger): Scale the trajectories instead of copying the last
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
      } else if (!trajectory.samples.empty()) {
        sample.pose(i) = trajectory.samples.back().pos;
        sample.vel(i) = trajectory.samples.back().vel;
        sample.accel(i) = trajectory.samples.back().accel;
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
}  // namespace BSplineWrapper
