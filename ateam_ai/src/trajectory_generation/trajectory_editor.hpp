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

#ifndef TRAJECTORY_GENERATION__TRAJECTORY_EDITOR_HPP_
#define TRAJECTORY_GENERATION__TRAJECTORY_EDITOR_HPP_

#include "behavior/behavior_feedback.hpp"
#include "types/robot.hpp"

namespace trajectory_editor
{
/**
 * Given a trajectory, crop the trajectory such that it starts at |new_start_time| and ends at |new_end_time|. All
 * samples outside this period are removed.
 */
Trajectory crop_trajectory(const Trajectory & a, double new_start_time, double new_end_time);

/**
 * Given two trajectories |a| and |b|, place |a| before |b|, with no editing of sample time
 */
Trajectory append_trajectory(const Trajectory & a, const Trajectory & b, double time_step);

/**
 * Prepends only the initial |immutable_duration| from |last_frame_trajectory| in front of |this_frame_trajectory|
 */
Trajectory apply_immutable_duration(
  const Trajectory & last_frame_trajectory, const Trajectory & this_frame_trajectory,
  double immutable_duration, double time_step, double current_time);

/**
 * Returns the robot state at the last sample before we can change our plan
 */
Robot state_at_immutable_duration(
  const Trajectory & last_frame_trajectory, double immutable_duration, double current_time);
}  // namespace trajectory_editor

#endif  // TRAJECTORY_GENERATION__TRAJECTORY_EDITOR_HPP_
