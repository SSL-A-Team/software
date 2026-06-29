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

std::array<std::optional<PathPlanResult>, 16> Planner::PlanPathsForAllBots(
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

  std::array<std::optional<PathPlanResult>, 16> paths;
  for (int bot_index : bot_indices) {
    if (!targets[bot_index].has_value()) {
      continue;
    }
    const auto & bot = world.our_robots.at(bot_index);
    std::vector<Obstacle> bot_obstacles = obstacles;
    bot_obstacles.insert(
      bot_obstacles.end(), per_bot_obstacles[bot_index].begin(),
        per_bot_obstacles[bot_index].end());

    if(options[bot_index].ignore_start_obstacles) {
      RemoveInitialCollidingObstacles(bot_obstacles, bot);
    }

    const auto should_replan = ShouldReplan(bot, targets[bot_index].value(), bot_obstacles,
        options[bot_index], world);
    if(should_replan) {
      paths[bot_index] = PlanPath(bot, targets[bot_index].value(), bot_obstacles,
          options[bot_index], world);
      if(paths[bot_index].has_value()) {
        cache_[bot_index] = CacheEntry{
          .trajectory = paths[bot_index].value().path,
          .options = options[bot_index],
          .target = targets[bot_index].value()
        };
      } else {
        cache_[bot_index].reset();
      }
    } else {
      const auto & cached_trajectory = cache_[bot_index]->trajectory;
      const auto elapsed =
        std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() -
          cached_trajectory.GetStartTime()).count();
      const auto collision_stats = collisions::GetCollisionStats(cached_trajectory, bot_obstacles,
          world, options[bot_index].collision_check_resolution,
          options[bot_index].collision_check_horizon, options[bot_index].footprint_inflation, elapsed);
      paths[bot_index] = PathPlanResult{
        cached_trajectory,
        collision_stats
      };
    }
    if(paths[bot_index].has_value()) {
      const auto obstacle_trajectory_time_step = options[bot_index].collision_check_resolution;
      const auto obstacle_trajectory =
        paths[bot_index]->path.ToPoints(obstacle_trajectory_time_step);
      obstacles.push_back(Obstacle::FromRobot(bot, obstacle_trajectory,
          obstacle_trajectory_time_step));
    } else {
      obstacles.push_back(Obstacle::FromRobot(bot));
    }
  }
  return paths;
}

std::optional<PathPlanResult> Planner::PlanPath(
  const ateam_game_state::Robot & robot, const Pose & target,
  const std::vector<Obstacle> & obstacles, const PlannerOptions & options,
  const ateam_game_state::World & world)
{
  const auto init_state = Vector6FromRobot(robot);
  const auto maybe_truncated_target = GetTruncatedTarget(robot, target, obstacles, options, world);
  if(!maybe_truncated_target.has_value()) {
    return std::nullopt;
  }
  const auto truncated_target = *maybe_truncated_target;
  const auto target_state = Vector3FromPose(truncated_target);
  const auto start_time = std::chrono::steady_clock::now();

  const auto trajectory_params = BuildTrajectoryParams(options.limits);

  BangBangTraj3D_t direct_trajectory;
  try {
    direct_trajectory = GenerateTrajectory(init_state, target_state, trajectory_params);
  } catch (const ControlsException &) {
    return std::nullopt;
  }

  const auto direct_path = MakeTrajectorySpline({
      .start_time = start_time,
      .start_state = init_state,
      .trajectory_params = trajectory_params,
      .segments = {{GetBangBangTrajectoryDuration(direct_trajectory), truncated_target,
        direct_trajectory}}
  });

  const auto direct_collision_stats = collisions::GetCollisionStats(direct_path, obstacles, world,
      options.collision_check_resolution, options.collision_check_horizon,
      options.footprint_inflation, 0.0);

  if (!direct_collision_stats.HasCollision()) {
    return PathPlanResult{
      direct_path,
      direct_collision_stats
    };
  }

  TrajectorySpline best_path = direct_path;
  auto best_path_collision_stats = direct_collision_stats;

  for(auto inter_target_dist = options.inter_target_dist_min;
    inter_target_dist <= options.inter_target_dist_max;
    inter_target_dist += options.inter_target_dist_step)
  {
    const auto inter_target_angle_offset_limit = M_PI_2 - std::fmod(M_PI_2,
        options.inter_target_angle_step);
    const auto angle_to_target = ateam_geometry::ToHeading(truncated_target.position - robot.pos);
    for(auto inter_target_angle_offset = -inter_target_angle_offset_limit;
      inter_target_angle_offset <= inter_target_angle_offset_limit;
      inter_target_angle_offset += options.inter_target_angle_step)
    {
      try {
        const auto inter_target_angle = angle_to_target + inter_target_angle_offset;
        const Pose inter_target {
          robot.pos +
          (ateam_geometry::directionFromAngle(inter_target_angle).vector() * inter_target_dist),
          truncated_target.heading
        };
        const auto inter_target_state = Vector3FromPose(inter_target);
        const auto sub_path_1 = GenerateTrajectory(init_state, inter_target_state,
            trajectory_params);

        const auto sub_path_1_duration = GetBangBangTrajectoryDuration(sub_path_1);
        for(auto transition_time = 0.1; transition_time < sub_path_1_duration;
          transition_time += 0.2)
        {
          const auto transition_state = GetStateAtT(sub_path_1, transition_time);
          const auto sub_path_2 = GenerateTrajectory(transition_state, target_state,
              trajectory_params);
          const auto sub_path_2_duration = GetBangBangTrajectoryDuration(sub_path_2);

          const auto candidate_path = MakeTrajectorySpline({
              .start_time = start_time,
              .start_state = init_state,
              .trajectory_params = trajectory_params,
              .segments = {
                {transition_time, inter_target, sub_path_1},
                {sub_path_2_duration, truncated_target, sub_path_2}
              }
          });

          const auto candidate_collision_stats = collisions::GetCollisionStats(candidate_path,
              obstacles, world, options.collision_check_resolution, options.collision_check_horizon,
              options.footprint_inflation, 0.0);

          if(ComparePaths(candidate_path, candidate_collision_stats, best_path,
              best_path_collision_stats) == std::partial_ordering::greater)
          {
            best_path = candidate_path;
            best_path_collision_stats = candidate_collision_stats;
          }
        }
      } catch (const ControlsException & e) {
        continue;
      }
    }
  }

  return PathPlanResult {
    best_path,
    best_path_collision_stats
  };
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

  const auto elapsed =
        std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() -
          cached_path.GetStartTime()).count();
  const auto collision_stats = collisions::GetCollisionStats(cached_path, obstacles, world,
      options.collision_check_resolution, options.collision_check_horizon,
      options.footprint_inflation, elapsed);

  if(collision_stats.HasCollision()) {
    return true;
  }

  return false;
}

std::optional<Pose> Planner::GetTruncatedTarget(
  const ateam_game_state::Robot & robot, const Pose & target,
  const std::vector<Obstacle> & obstacles, const PlannerOptions & options,
  const ateam_game_state::World & world)
{
  const auto step_vector = ateam_geometry::normalize(robot.pos - target.position) *
    options.collision_check_resolution;
  const auto step_count = ateam_geometry::norm(robot.pos - target.position) /
    options.collision_check_resolution;
  auto candidate = target.position;
  for(auto s = 0; s < step_count; ++s) {
    if(!collisions::DoesPointCollideWithObstacles(candidate, 0.0, obstacles,
        options.footprint_inflation) &&
      collisions::IsPointInBounds(candidate, world, options.footprint_inflation))
    {
      return Pose{
        .position = candidate,
        .heading = target.heading
      };
    }
    candidate += step_vector;
  }
  return std::nullopt;
}

void Planner::RemoveInitialCollidingObstacles(
  std::vector<Obstacle> & obstacles,
  const ateam_game_state::Robot & robot)
{
  const auto robot_footprint = ateam_geometry::makeDisk(robot.pos, kRobotRadius);
  const auto new_end = std::remove_if(obstacles.begin(), obstacles.end(),
      [&robot_footprint](const auto & obstacle){
        return ateam_geometry::doIntersect(robot_footprint, obstacle.shape);
  });
  obstacles.erase(new_end, obstacles.end());
}

std::partial_ordering Planner::ComparePaths(
  const TrajectorySpline & path_l, const CollisionStats & stats_l,
  const TrajectorySpline & path_r, const CollisionStats & stats_r)
{
  // const auto collision_free_time = [](const TrajectorySpline & path, const CollisionStats & stats){
  //     const auto init_collision_end = stats.init_collision_end_time.value_or(0.0);
  //     const auto new_collision_start =
  //       stats.new_collision_start_time.value_or(path.GetTotalDuration());
  //     return new_collision_start - init_collision_end;
  //   };
  if(stats_l.HasCollision()) {
    if(stats_r.HasCollision()) {
      // if(auto cmp = collision_free_time(path_l, stats_l) <=> collision_free_time(path_r, stats_r);
      //   cmp != std::partial_ordering::equivalent)
      // {
      //   return cmp;
      // }
      // if(auto cmp = stats_l.init_collision_end_time.value_or(0.0) <=>
      //   stats_r.init_collision_end_time.value_or(0.0); cmp != std::partial_ordering::equivalent)
      // {
      //   return cmp;
      // }
      // Compare total durations with opposite l/r because lower durations are better
      return path_r.GetTotalDuration() <=> path_l.GetTotalDuration();
    } else {
      return std::partial_ordering::less;
    }
  } else {
    if(stats_r.HasCollision()) {
      return std::partial_ordering::greater;
    } else {
      return path_r.GetTotalDuration() <=> path_l.GetTotalDuration();
    }
  }
}

}  // namespace ateam_path_planning
