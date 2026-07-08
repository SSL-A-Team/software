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

#ifndef ATEAM_PATH_PLANNING__COLLIISIONS_HPP_
#define ATEAM_PATH_PLANNING__COLLIISIONS_HPP_

#include <ateam_controls/ateam_controls.h>
#include <vector>
#include <ateam_game_state/world.hpp>
#include "obstacle.hpp"
#include "trajectory_spline.hpp"
#include "collision_stats.hpp"

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
  const double boundary_footprint_inflation,
  const double search_start_t);

CollisionStats GetCollisionStats(
  const TrajectorySpline & spline,
  const std::vector<Obstacle> & obstacles,
  const ateam_game_state::World & world,
  const double collision_check_resolution,
  const double collision_check_horizon,
  const double footprint_inflation,
  const double boundary_footprint_inflation,
  const double search_start_t);

bool DoesPointCollideWithObstacles(
  const ateam_geometry::Point & point,
  const double & t,
  const std::vector<Obstacle> & obstacles,
  const double footprint_inflation);

bool IsPointInBounds(
  const ateam_geometry::Point & point, const ateam_game_state::World & world,
  const double footprint_inflation);

bool DoesStateCollideWithObstacles(
  const Vector6C_t & state,
  const double & t,
  const std::vector<Obstacle> & obstacles,
  const double footprint_inflation);

bool IsStateInBounds(
  const Vector6C_t & state, const ateam_game_state::World & world,
  const double footprint_inflation);

}  // namespace ateam_path_planning::collisions

#endif  // ATEAM_PATH_PLANNING__COLLIISIONS_HPP_
