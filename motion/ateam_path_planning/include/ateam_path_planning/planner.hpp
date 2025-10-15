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

#ifndef ATEAM_PATH_PLANNING__PLANNER_HPP_
#define ATEAM_PATH_PLANNING__PLANNER_HPP_

#include <array>
#include <optional>
#include <vector>
#include <ateam_game_state/world.hpp>
#include <ateam_geometry/types.hpp>
#include "obstacle.hpp"
#include "pose.hpp"
#include "trajectory_spline.hpp"

namespace ateam_path_planning
{

class Planner {
public:
  Planner() = default;

  std::array<std::optional<TrajectorySpline>, 16> PlanPathsForAllBots(
    const std::array<std::optional<Pose>, 16> & targets, const std::array<uint8_t, 16> & priorities,
    const ateam_game_state::World & world, const std::vector<Obstacle> & global_obstacles,
    const std::array<std::vector<Obstacle>, 16> & per_bot_obstacles);

  std::optional<TrajectorySpline> PlanPath(
    const Pose & start_pose, const ateam_geometry::Vector & start_velocity,
    const Pose & target, const std::vector<Obstacle> & obstacles);

private:
};
}  // namespace ateam_path_planning

#endif  // ATEAM_PATH_PLANNING__PLANNER_HPP_
