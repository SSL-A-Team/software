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

#include <gtest/gtest.h>

#include <vector>

testing::AssertionResult IsValidTrajectory(
  TrapezoidalMotionProfile::Trajectory1d trajectory,
  double max_vel, double max_accel, double dt)
{
  // Pos difference matches vel
  for (std::size_t i = 0; i < trajectory.samples.size() - 1; i++) {
    const auto & cur_sample = trajectory.samples.at(i);
    const auto & next_sample = trajectory.samples.at(i + 1);
    double sample_vel = (next_sample.pos - cur_sample.pos) / dt;
    double avg_vel = (cur_sample.vel + next_sample.vel) / 2;
    if (std::abs(sample_vel - avg_vel) > max_vel + 1e-3) {
      return testing::AssertionFailure() << sample_vel << " (" << next_sample.pos <<
             " - " << cur_sample.pos << ") pos difference doesn't match average vel " <<
             avg_vel << " at T=" << cur_sample.time;
    }
  }

  // Vel difference matches accel
  for (std::size_t i = 0; i < trajectory.samples.size() - 1; i++) {
    const auto & cur_sample = trajectory.samples.at(i);
    const auto & next_sample = trajectory.samples.at(i + 1);
    double sample_accel = (next_sample.vel - cur_sample.vel) / dt;
    double avg_accel = (cur_sample.accel + next_sample.accel) / 2;
    if (std::abs(sample_accel - avg_accel) > max_accel + 1e-3) {
      return testing::AssertionFailure() << sample_accel << " (" << next_sample.vel <<
             " - " << cur_sample.vel << ") vel difference doesn't match avg accel " <<
             avg_accel << " at T=" << cur_sample.time;
    }
  }

  return testing::AssertionSuccess();
}

TEST(TrapezoidalMotionProfile, Trajectory1d_ShouldReturnValidTrajectory_WhenSimpleInput)
{
  double start = 0;
  double start_vel = 0;
  double end = 10;
  double end_vel = 0;
  double max_vel = 1;
  double max_accel = 1;
  double dt = 0.1;

  TrapezoidalMotionProfile::Trajectory1d ret = TrapezoidalMotionProfile::Generate1d(
    start,
    start_vel,
    end,
    end_vel,
    max_vel,
    max_accel,
    dt);

  ASSERT_TRUE(IsValidTrajectory(ret, max_vel, max_accel, dt));
}

TEST(TrapezoidalMotionProfile, Trajectory1d_ShouldReturnValidTrajectory_WhenNegativeInput)
{
  double start = 10;
  double start_vel = 0;
  double end = 0;
  double end_vel = 0;
  double max_vel = 1;
  double max_accel = 1;
  double dt = 0.1;

  TrapezoidalMotionProfile::Trajectory1d ret = TrapezoidalMotionProfile::Generate1d(
    start,
    start_vel,
    end,
    end_vel,
    max_vel,
    max_accel,
    dt);

  ASSERT_TRUE(IsValidTrajectory(ret, max_vel, max_accel, dt));
}

TEST(TrapezoidalMotionProfile, Trajectory1d_ShouldReturnValidTrajectory_WhenNegativeSpeed)
{
  double start = 0;
  double start_vel = -1;
  double end = 10;
  double end_vel = 0;
  double max_vel = 1;
  double max_accel = 1;
  double dt = 0.1;

  TrapezoidalMotionProfile::Trajectory1d ret = TrapezoidalMotionProfile::Generate1d(
    start,
    start_vel,
    end,
    end_vel,
    max_vel,
    max_accel,
    dt);

  ASSERT_TRUE(IsValidTrajectory(ret, max_vel, max_accel, dt));
}

TEST(TrapezoidalMotionProfile, Trajectory1d_ShouldReturnValidTrajectory_WhenShort)
{
  double start = 0;
  double start_vel = 0;
  double end = 1;
  double end_vel = 0;
  double max_vel = 1;
  double max_accel = 1;
  double dt = 0.1;

  TrapezoidalMotionProfile::Trajectory1d ret = TrapezoidalMotionProfile::Generate1d(
    start,
    start_vel,
    end,
    end_vel,
    max_vel,
    max_accel,
    dt);

  ASSERT_TRUE(IsValidTrajectory(ret, max_vel, max_accel, dt));
}

TEST(TrapezoidalMotionProfile, Trajectory1d_ShouldReturnValidTrajectory_WhenShortNegative)
{
  double start = 0;
  double start_vel = -1;
  double end = 0;
  double end_vel = 0;
  double max_vel = 1;
  double max_accel = 1;
  double dt = 0.1;

  TrapezoidalMotionProfile::Trajectory1d ret = TrapezoidalMotionProfile::Generate1d(
    start,
    start_vel,
    end,
    end_vel,
    max_vel,
    max_accel,
    dt);

  ASSERT_TRUE(IsValidTrajectory(ret, max_vel, max_accel, dt));
}
