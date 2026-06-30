// Copyright 2026 A Team
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

CollisionStats GetCollisionStats(
  const BangBangTraj3D_t & trajectory,
  const double traj_start_t,
  const std::vector<Obstacle> & obstacles,
  const ateam_game_state::World & world,
  const double collision_check_resolution,
  const double collision_check_horizon,
  const double footprint_inflation,
  const double search_start_t)
{
  const auto duration = std::min(GetBangBangTrajectoryDuration(trajectory),
      collision_check_horizon);
  const auto init_state = GetStateAtT(trajectory, 0.0);
  bool was_in_collision = DoesStateCollideWithObstacles(init_state, traj_start_t, obstacles,
      footprint_inflation);
  CollisionStats stats;
  const auto t0 = std::max(search_start_t, collision_check_resolution);
  for (double t = t0; t < duration; t += collision_check_resolution) {
    const auto state_at_t = GetStateAtT(trajectory, t);
    const auto is_colliding = DoesStateCollideWithObstacles(state_at_t, t + traj_start_t, obstacles,
        footprint_inflation);
    if(was_in_collision && !is_colliding) {
      stats.init_collision_end_time = t + traj_start_t;
    }
    if(!was_in_collision && is_colliding) {
      stats.new_collision_start_time = t + traj_start_t;
      return stats;
    }
    if(!IsStateInBounds(state_at_t, world, footprint_inflation)) {
      stats.new_collision_start_time = t + traj_start_t;
      return stats;
    }
    was_in_collision = is_colliding;
  }

  return stats;
}

CollisionStats GetCollisionStats(
  const TrajectorySpline & spline,
  const std::vector<Obstacle> & obstacles,
  const ateam_game_state::World & world,
  const double collision_check_resolution,
  const double collision_check_horizon,
  const double footprint_inflation,
  const double search_start_t)
{
  CollisionStats stats;
  if(spline.GetSegmentCount() == 0) {
    return stats;
  }
  double path_t = 0.0;
  const auto init_state = Vector6FromPose(spline.GetStartPose());
  bool was_in_collision = DoesStateCollideWithObstacles(init_state, 0.0, obstacles,
      footprint_inflation);
  for(const auto & segment : spline.impl_->segments) {
    if(path_t >= collision_check_horizon) {
      return stats;
    }
    if(path_t + segment.duration < search_start_t) {
      path_t += segment.duration;
      continue;
    }
    const auto horizon = std::min(collision_check_horizon - path_t, segment.duration);
    const auto segment_search_start_t = std::max(search_start_t - path_t, 0.0);
    const auto segment_collision_stats = GetCollisionStats(segment.trajectory, path_t, obstacles,
        world, collision_check_resolution, horizon, footprint_inflation, segment_search_start_t);
    if(segment_collision_stats.init_collision_end_time.has_value()) {
      if(was_in_collision) {
        stats.init_collision_end_time = segment_collision_stats.init_collision_end_time;
        was_in_collision = false;
      } else {
        stats.new_collision_start_time = path_t;
        return stats;
      }
    }
    if(segment_collision_stats.new_collision_start_time.has_value()) {
      stats.new_collision_start_time = segment_collision_stats.new_collision_start_time;
      return stats;
    }
    path_t += segment.duration;
  }
  return stats;
}

bool DoesPointCollideWithObstacles(
  const ateam_geometry::Point & point,
  const double & t,
  const std::vector<Obstacle> & obstacles,
  const double footprint_inflation)
{
  const auto robot_footprint = ateam_geometry::makeDisk(point,
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

bool IsPointInBounds(
  const ateam_geometry::Point & point, const ateam_game_state::World & world,
  const double footprint_inflation)
{
  const auto x = point.x();
  const auto y = point.y();
  if((std::fabs(x) + kRobotRadius + footprint_inflation) >=
    ((world.field.field_length / 2.0) + world.field.boundary_width))
  {
    return false;
  }
  if((std::fabs(y) + kRobotRadius + footprint_inflation) >=
    ((world.field.field_width / 2.0) + world.field.boundary_width))
  {
    return false;
  }
  return true;
}

bool DoesStateCollideWithObstacles(
  const Vector6C_t & state,
  const double & t,
  const std::vector<Obstacle> & obstacles,
  const double footprint_inflation)
{
  const ateam_geometry::Point robot_pos(state.data[0], state.data[1]);
  return DoesPointCollideWithObstacles(robot_pos, t, obstacles, footprint_inflation);
}

bool IsStateInBounds(
  const Vector6C_t & state, const ateam_game_state::World & world,
  const double footprint_inflation)
{
  const auto x = state.data[0];
  const auto y = state.data[1];
  if((std::fabs(x) + kRobotRadius + footprint_inflation) >=
    ((world.field.field_length / 2.0) + world.field.boundary_width))
  {
    return false;
  }
  if((std::fabs(y) + kRobotRadius + footprint_inflation) >=
    ((world.field.field_width / 2.0) + world.field.boundary_width))
  {
    return false;
  }
  return true;
}

}  // namespace ateam_path_planning::collisions
