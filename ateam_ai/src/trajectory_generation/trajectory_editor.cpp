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

#include "trajectory_generation/trajectory_editor.hpp"

Trajectory trajectory_editor::crop_trajectory(
  const Trajectory & a, double new_start_time,
  double new_end_time)
{
  Trajectory output;
  std::copy_if(
    a.samples.begin(), a.samples.end(), std::back_inserter(output.samples),
    [new_start_time, new_end_time](Sample3d sample) {
      return sample.time >= new_start_time && sample.time < new_end_time;
    });

  return output;
}

Trajectory trajectory_editor::append_trajectory(
  const Trajectory & a, const Trajectory & b,
  double /**time_step**/)
{
  Trajectory output = a;

  // double final_a_time = a.samples.empty() ? 0 : a.samples.back().time;
  // double initial_b_time = b.samples.empty() ? 0 : b.samples.front().time;
  for (auto sample : b.samples) {
    // sample.time = sample.time - initial_b_time + time_step + final_a_time;
    output.samples.push_back(sample);
  }

  return output;
}

Trajectory trajectory_editor::apply_immutable_duration(
  const Trajectory & last_frame_trajectory,
  const Trajectory & this_frame_trajectory,
  double immutable_duration,
  double time_step,
  double current_time)
{
  Trajectory first = crop_trajectory(
    last_frame_trajectory, current_time,
    current_time + immutable_duration);
  Trajectory output = append_trajectory(first, this_frame_trajectory, time_step);

  return output;
}

Robot trajectory_editor::state_at_immutable_duration(
  const Trajectory & last_frame_trajectory, double immutable_duration,
  double current_time)
{
  assert(!last_frame_trajectory.samples.empty());

  Sample3d state = last_frame_trajectory.samples.front();
  for (const auto & sample : last_frame_trajectory.samples) {
    if (sample.time <= current_time + immutable_duration) {
      state = sample;
    } else {
      break;
    }
  }

  Robot output;
  output.pos = Eigen::Vector2d{state.pose.x(), state.pose.y()};
  output.theta = state.pose.z();
  output.vel = Eigen::Vector2d{state.vel.x(), state.vel.y()};
  output.omega = state.vel.z();

  return output;
}
