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


#ifndef CORE__MOTION__PATH_PLANNING__PATH_PLANNER_HPP_
#define CORE__MOTION__PATH_PLANNING__PATH_PLANNER_HPP_

#include <memory>
#include <vector>
#include <ateam_common/robot_constants.hpp>
#include <ateam_geometry/any_shape.hpp>
#include <ateam_geometry/types.hpp>
#include "core/types/state_types.hpp"
#include "core/motion/motion_command.hpp"
#include "planner_options.hpp"
#include "path_planning_target.hpp"
#include "core/visualization/overlays.hpp"

namespace ateam_path_planning
{
class Planner;
class Obstacle;
class TrajectorySpline;
}

namespace ateam_kenobi::motion::path_planning
{

class PathPlanner
{
public:
  PathPlanner();

  ~PathPlanner();

  void Execute(
    std::array<std::optional<MotionCommand>, 16> & commands,
    const std::vector<PathPlanningTarget> & targets, const World & world,
    visualization::Overlays & overlays);

private:
  std::unique_ptr<ateam_path_planning::Planner> planner_;

  void DrawObstacles(
    visualization::Overlays & overlays,
    const std::vector<ateam_path_planning::Obstacle> & obstacles);

  void DrawTrajectory(
    visualization::Overlays & overlays,
    const std::optional<ateam_path_planning::TrajectorySpline> & maybe_path, const Robot & robot,
    const ateam_geometry::Point & target, const PlannerOptions & options);
};

}  // namespace ateam_kenobi::motion::path_planning

#endif  // CORE__MOTION__PATH_PLANNING__PATH_PLANNER_HPP_
