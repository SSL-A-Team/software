// Copyright 2025 A Team
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

#include "ateam_path_planning/colliisions.hpp"
#include <ateam_common/robot_constants.hpp>
#include <ateam_geometry/do_intersect.hpp>
#include <ateam_geometry/creation_helpers.hpp>
#include <ateam_geometry/printing.hpp>
#include "controls_lib_adapters.hpp"
#include "trajectory_spline_impl.hpp"

namespace ateam_path_planning::collisions
{

// TODO(barulicm): check state validity (in bounds)

std::optional<double> TimeToCollision(
  const BangBangTraj3D & trajectory,
  const double & start_t,
  const std::vector<Obstacle> & obstacles,
  const ateam_game_state::World & world,
  const double collision_check_resolution,
  const double collision_check_horizon,
  const double footprint_inflation)
{
  const auto duration = std::min(GetBangBangTrajectoryDuration(trajectory),
      collision_check_horizon);
  for (double t = 0.0; t < duration; t += collision_check_resolution) {
    Vector6C_t state_at_t;
    if(const auto err =
      ateam_controls_traj_state_at(trajectory, t, &state_at_t);
      err != ATEAM_CONTROLS_OK)
    {
      throw ControlsException(err);
    }
    if(DoesStateCollideWithObstacles(state_at_t, t + start_t, obstacles, footprint_inflation)) {
      return t;
    }
    if(!IsStateInBounds(state_at_t, world, footprint_inflation)) {
      return t;
    }
  }

  return std::nullopt;
}

std::optional<double> TimeToCollision(
  const TrajectorySpline & spline,
  const std::vector<Obstacle> & obstacles,
  const ateam_game_state::World & world,
  const double collision_check_resolution,
  const double collision_check_horizon,
  const double footprint_inflation)
{
  double path_t = 0.0;
  for(const auto & segment : spline.impl_->segments) {
    if(path_t >= collision_check_horizon) {
      return std::nullopt;
    }
    const auto horizon = std::min(collision_check_horizon - path_t, segment.duration);
    const auto segment_collision_time = TimeToCollision(segment.trajectory, path_t, obstacles,
        world, collision_check_resolution, horizon, footprint_inflation);
    if(segment_collision_time.has_value()) {
      return segment_collision_time;
    }
    path_t += segment.duration;
  }
  return std::nullopt;
}

bool DoesStateCollideWithObstacles(
  const Vector6C_t & state,
  const double & t,
  const std::vector<Obstacle> & obstacles,
  const double footprint_inflation)
{
  const ateam_geometry::Point robot_pos(state.data[0], state.data[1]);
  const auto robot_footprint = ateam_geometry::makeDisk(robot_pos,
        kRobotRadius + footprint_inflation);
  for (const auto & obstacle : obstacles) {
    if(ateam_geometry::doIntersect(robot_footprint,
          obstacle.ShapeAtT(t)))
    {
      return true;
    }
  }
  return false;
}

bool IsStateInBounds(
  const Vector6C_t & state, const ateam_game_state::World & world,
  const double footprint_inflation)
{
  const auto x = state.data[0];
  const auto y = state.data[1];
  if((std::fabs(x) + kRobotRadius + footprint_inflation) >= (world.field.field_length / 2.0)) {
    return false;
  }
  if((std::fabs(y) + kRobotRadius + footprint_inflation) >= (world.field.field_width / 2.0)) {
    return false;
  }
  return true;
}

}  // namespace ateam_path_planning::collisions
