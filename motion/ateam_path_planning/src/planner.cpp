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
#include "ateam_path_planning/controls_lib_adapters.hpp"

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
    auto path = PlanPath(bot, targets[bot_index].value(), bot_obstacles, options[bot_index]);
    paths[bot_index] = path;
    const auto obstacle_trajectory_time_step = options[bot_index].collision_check_resolution;
    const auto obstacle_trajectory = path->ToPoints(obstacle_trajectory_time_step);
    obstacles.push_back(Obstacle::FromRobot(bot, obstacle_trajectory,
        obstacle_trajectory_time_step));
  }
  return paths;
}

std::optional<TrajectorySpline> Planner::PlanPath(
  const ateam_game_state::Robot & robot, const Pose & target,
  const std::vector<Obstacle> & obstacles, const PlannerOptions & options)
{
  const auto init_state = Vector6FromRobot(robot);
  const auto target_state = Vector3FromPose(target);

  const auto trajectory_params = ateam_controls_default_traj_params();

  BangBangTraj3D_t base_trajectory;
  if(const auto err = ateam_controls_traj_from_target_pose(init_state, target_state,
      trajectory_params, &base_trajectory); err != ATEAM_CONTROLS_OK)
  {
    return std::nullopt;
  }

  const auto collision_time = collisions::TimeToCollision(base_trajectory, 0.0,
      obstacles, options.collision_check_resolution, options.collision_check_horizon,
      options.footprint_inflation);

  if (!collision_time.has_value()) {
    TrajectorySpline result;
    result.start_pose.position = robot.pos;
    result.start_pose.heading = robot.theta;
    result.start_velocity = robot.vel;
    result.segments.emplace_back(GetBangBangTrajectoryDuration(base_trajectory), target);
    return result;
  }

  auto fastest_time = std::numeric_limits<double>::infinity();
  TrajectorySpline fastest_trajectory;
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
        return std::nullopt;
      }
      const auto inter_collision_time = collisions::TimeToCollision(inter_traj, 0.0,
          obstacles, options.collision_check_resolution, options.collision_check_horizon,
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
          return std::nullopt;
        }
        const auto second_collision_time = collisions::TimeToCollision(second_traj,
            transition_time, obstacles, options.collision_check_resolution,
            options.collision_check_horizon,
            options.footprint_inflation);
        if (!second_collision_time.has_value()) {
          const auto second_traj_duration = GetBangBangTrajectoryDuration(second_traj);
          const auto total_time = transition_time + second_traj_duration;
          if (total_time < fastest_time) {
            fastest_time = total_time;
            TrajectorySpline result;
            result.start_pose.position = robot.pos;
            result.start_pose.heading = robot.theta;
            result.start_velocity = robot.vel;
            result.segments.emplace_back(transition_time, inter_target);
            result.segments.emplace_back(second_traj_duration, target);
            fastest_trajectory = result;
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

}  // namespace ateam_path_planning
