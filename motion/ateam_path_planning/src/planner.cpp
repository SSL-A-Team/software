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
#include <algorithm>
#include <numeric>
#include <ateam_common/robot_constants.hpp>
#include <ateam_geometry/creation_helpers.hpp>
#include <ateam_controls/ateam_controls.h>
#include "ateam_path_planning/colliisions.hpp"
#include "ateam_path_planning/controls_lib_adapters.hpp"

namespace ateam_path_planning
{

std::array<std::optional<TrajectorySpline>, 16> Planner::PlanPathsForAllBots(
  const std::array<std::optional<Pose>, 16> & targets, const std::array<uint8_t, 16> & priorities,
  const ateam_game_state::World & world, const std::vector<Obstacle> & global_obstacles,
  const std::array<std::vector<Obstacle>, 16> & per_bot_obstacles)
{
  std::array<int, 16> bot_indices;
  std::iota(bot_indices.begin(), bot_indices.end(), 0);
  std::ranges::sort(bot_indices, [&priorities](int a, int b) {
      return priorities[a] > priorities[b];
    });

  std::vector<Obstacle> obstacles = global_obstacles;
  obstacles.insert(obstacles.end(), global_obstacles.begin(), global_obstacles.end());
  std::ranges::transform(world.our_robots, std::back_inserter(obstacles), Obstacle::FromRobot);

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
    auto path = PlanPath(bot, targets[bot_index].value(), bot_obstacles);
    paths[bot_index] = path;
    obstacles.push_back(Obstacle::FromRobot(bot));
  }
  return paths;
}

std::optional<TrajectorySpline> Planner::PlanPath(
  const ateam_game_state::Robot & robot, const Pose & target,
    const std::vector<Obstacle> & obstacles)
{
  RigidBodyState init_state = RigidBodyStateFromRobot(robot);
  RigidBodyState target_state = RigidBodyStateFromPose(target);

  const auto base_trajectory = ateam_controls_compute_optimal_bangbang_traj_3d(init_state, target_state);

  const auto collision_time = collisions::TimeToCollision(base_trajectory, obstacles);
  
  if (!collision_time.has_value()) {
    TrajectorySpline result;
    result.segments.emplace_back(0.0, target);
    return result;
  }

  auto fastest_time = std::numeric_limits<double>::infinity();
  TrajectorySpline fastest_trajectory;
  bool found_collision_free = false;

  for(auto inter_target_dist = 1.0; inter_target_dist < 5.0; inter_target_dist += 1.0) {
    for(auto inter_target_angle = 0.0; inter_target_angle < M_2_PI; inter_target_angle += M_PI_4) {
      const Pose inter_target {
        robot.pos + (ateam_geometry::directionFromAngle(inter_target_angle).vector() * inter_target_dist),
        target.heading
      };
      const auto inter_target_state = RigidBodyStateFromPose(inter_target);
      const auto inter_traj = ateam_controls_compute_optimal_bangbang_traj_3d(init_state, inter_target_state);
      const auto inter_collision_time = collisions::TimeToCollision(inter_traj, obstacles);
      const auto max_time = inter_collision_time.value_or(GetBangBangTrajectoryDuration(inter_traj));

      for(auto transition_time = 0.1; transition_time < max_time; transition_time += 0.1) {
        const auto transition_state = ateam_controls_compute_bangbang_traj_3d_state_at_t(inter_traj, init_state, 0.0, transition_time);
        const auto second_traj = ateam_controls_compute_optimal_bangbang_traj_3d(transition_state, target_state);
        const auto second_collision_time = collisions::TimeToCollision(second_traj, obstacles);
        if (!second_collision_time.has_value()) {
          const double total_time = transition_time + GetBangBangTrajectoryDuration(second_traj);
          if (total_time < fastest_time) {
            fastest_time = total_time;
            TrajectorySpline result;
            result.segments.emplace_back(0.0, inter_target);
            result.segments.emplace_back(transition_time, target);
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
