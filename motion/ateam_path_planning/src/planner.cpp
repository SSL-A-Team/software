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

#include "ateam_path_planning/planner.hpp"
#include <ateam_controls/ateam_controls.h>
#include <algorithm>
#include <numeric>
#include <ateam_common/robot_constants.hpp>
#include <ateam_geometry/creation_helpers.hpp>
#include "ateam_path_planning/colliisions.hpp"
#include "controls_lib_adapters.hpp"
#include "trajectory_spline_impl.hpp"

namespace ateam_path_planning
{

std::array<std::optional<TrajectorySpline>, 16> Planner::PlanPathsForAllBots(
  const std::array<std::optional<Pose>, 16> & targets, const std::array<uint8_t, 16> & priorities,
  const ateam_game_state::World & world, const std::vector<Obstacle> & global_obstacles,
  const std::array<std::vector<Obstacle>, 16> & per_bot_obstacles,
  const std::array<PlannerOptions, 16> & options)
{
  std::array<int, 16> bot_indices;
  std::iota(bot_indices.begin(), bot_indices.end(), 0);
  std::ranges::sort(bot_indices, [&priorities](int a, int b) {
      return priorities[a] > priorities[b];
    });

  std::vector<Obstacle> obstacles = global_obstacles;
  obstacles.insert(obstacles.end(), global_obstacles.begin(), global_obstacles.end());

  std::array<std::optional<TrajectorySpline>, 16> paths;
  for (int bot_index : bot_indices) {
    if (!targets[bot_index].has_value()) {
      continue;
    }
    const auto & bot = world.our_robots.at(bot_index);
    std::vector<Obstacle> bot_obstacles = obstacles;
    bot_obstacles.insert(
      bot_obstacles.end(), per_bot_obstacles[bot_index].begin(),
        per_bot_obstacles[bot_index].end());

    const auto should_replan = ShouldReplan(bot, targets[bot_index].value(), bot_obstacles,
        options[bot_index], world);
    if(should_replan) {
      paths[bot_index] = PlanPath(bot, targets[bot_index].value(), bot_obstacles,
          options[bot_index], world);
      cache_[bot_index] = CacheEntry{
        .trajectory = paths[bot_index].value(),
        .options = options[bot_index],
        .target = targets[bot_index].value()
      };
    } else {
      paths[bot_index] = cache_[bot_index]->trajectory;
    }
    if(paths[bot_index].has_value()) {
      const auto obstacle_trajectory_time_step = options[bot_index].collision_check_resolution;
      const auto obstacle_trajectory = paths[bot_index]->ToPoints(obstacle_trajectory_time_step);
      obstacles.push_back(Obstacle::FromRobot(bot, obstacle_trajectory,
          obstacle_trajectory_time_step));
    } else {
      obstacles.push_back(Obstacle::FromRobot(bot));
    }
  }
  return paths;
}

std::optional<TrajectorySpline> Planner::PlanPath(
  const ateam_game_state::Robot & robot, const Pose & target,
  const std::vector<Obstacle> & obstacles, const PlannerOptions & options,
  const ateam_game_state::World & world)
{
  const auto init_state = Vector6FromRobot(robot);
  const auto target_state = Vector3FromPose(target);
  const auto start_time = std::chrono::steady_clock::now();

  const auto trajectory_params = BuildTrajectoryParams(options.limits);

  BangBangTraj3D_t base_trajectory;
  if(const auto err = ateam_controls_traj_from_target_pose(init_state, target_state,
      trajectory_params, &base_trajectory); err != ATEAM_CONTROLS_OK)
  {
    throw ControlsException(err);
  }

  const auto collision_time = collisions::TimeToCollision(base_trajectory, 0.0,
      obstacles, world, options.collision_check_resolution, options.collision_check_horizon,
      options.footprint_inflation);

  if (!collision_time.has_value()) {
    TrajectorySplineImpl result;
    result.start_time = start_time;
    result.start_state = init_state;
    result.trajectory_params = trajectory_params;
    result.segments.emplace_back(GetBangBangTrajectoryDuration(base_trajectory), target,
        base_trajectory);
    return std::make_optional(MakeTrajectorySpline(result));
  }

  auto fastest_time = std::numeric_limits<double>::infinity();
  std::optional<TrajectorySpline> fastest_trajectory = std::nullopt;
  bool found_collision_free = false;

  for(auto inter_target_dist = options.inter_target_dist_min;
    inter_target_dist <= options.inter_target_dist_max;
    inter_target_dist += options.inter_target_dist_step)
  {
    const auto inter_target_angle_offset_limit = M_PI_2 - std::fmod(M_PI_2,
        options.inter_target_angle_step);
    const auto angle_to_target = ateam_geometry::ToHeading(target.position - robot.pos);
    for(auto inter_target_angle_offset = -inter_target_angle_offset_limit;
      inter_target_angle_offset <= inter_target_angle_offset_limit;
      inter_target_angle_offset += options.inter_target_angle_step)
    {
      const auto inter_target_angle = angle_to_target + inter_target_angle_offset;
      const Pose inter_target {
        robot.pos +
        (ateam_geometry::directionFromAngle(inter_target_angle).vector() * inter_target_dist),
        target.heading
      };
      const auto inter_target_state = Vector3FromPose(inter_target);
      BangBangTraj3D_t inter_traj;
      if(const auto err =
        ateam_controls_traj_from_target_pose(init_state, inter_target_state, trajectory_params,
          &inter_traj); err != ATEAM_CONTROLS_OK)
      {
        continue;
      }
      const auto inter_collision_time = collisions::TimeToCollision(inter_traj, 0.0,
          obstacles, world, options.collision_check_resolution, options.collision_check_horizon,
          options.footprint_inflation);
      const auto max_time =
        inter_collision_time.value_or(GetBangBangTrajectoryDuration(inter_traj));

      for(auto transition_time = 0.1; transition_time < max_time; transition_time += 0.1) {
        Vector6C_t transition_state;
        if (const auto err =
          ateam_controls_traj_state_at(inter_traj, transition_time,
            &transition_state); err != ATEAM_CONTROLS_OK)
        {
          continue;
        }
        BangBangTraj3D_t second_traj;
        if(const auto err =
          ateam_controls_traj_from_target_pose(transition_state, target_state, trajectory_params,
            &second_traj); err != ATEAM_CONTROLS_OK)
        {
          continue;
        }
        const auto second_collision_time = collisions::TimeToCollision(second_traj,
            transition_time, obstacles, world, options.collision_check_resolution,
            options.collision_check_horizon,
            options.footprint_inflation);
        if (!second_collision_time.has_value()) {
          const auto second_traj_duration = GetBangBangTrajectoryDuration(second_traj);
          const auto total_time = transition_time + second_traj_duration;
          if (total_time < fastest_time) {
            fastest_time = total_time;
            TrajectorySplineImpl result;
            result.start_time = start_time;
            result.start_state = init_state;
            result.trajectory_params = trajectory_params;
            result.segments.emplace_back(transition_time, inter_target, inter_traj);
            result.segments.emplace_back(second_traj_duration, target, second_traj);
            fastest_trajectory = MakeTrajectorySpline(result);
            found_collision_free = true;
          }
        }
      }
    }
  }

  if(!found_collision_free) {
    return std::nullopt;
  }

  return fastest_trajectory;
}

bool Planner::ShouldReplan(
  const ateam_game_state::Robot & robot, const Pose & target,
  const std::vector<Obstacle> & obstacles, const PlannerOptions & options,
  const ateam_game_state::World & world)
{
  const auto & cache = cache_[robot.id];
  if(!cache.has_value()) {
    return true;
  }

  if(cache->options != options) {
    return true;
  }

  if(CGAL::squared_distance(target.position,
      cache->target.position) >=
    (options.replan_thresholds.goal_distance * options.replan_thresholds.goal_distance))
  {
    return true;
  }

  const auto & cached_path = cache->trajectory;

  const auto check_time = std::chrono::steady_clock::now() -
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(
      options.replan_thresholds.lag_estimate));
  const auto expected_state = cached_path.GetStateAt(check_time);
  if(!expected_state.has_value()) {
    return true;
  }

  if(CGAL::squared_distance(robot.pos,
      expected_state->position) >=
    (options.replan_thresholds.deviation_distance * options.replan_thresholds.deviation_distance))
  {
    return true;
  }

  const auto collision_time = collisions::TimeToCollision(cached_path, obstacles, world,
      options.collision_check_resolution, options.collision_check_horizon,
      options.footprint_inflation);
  if(collision_time.has_value()) {
    return true;
  }

  return false;
}

}  // namespace ateam_path_planning
