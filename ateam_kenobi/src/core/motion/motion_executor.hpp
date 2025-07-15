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

#ifndef CORE__MOTION__MOTION_EXECUTOR_HPP_
#define CORE__MOTION__MOTION_EXECUTOR_HPP_

#include <array>
#include <optional>
#include <rclcpp/logger.hpp>
#include "core/path_planning/path_planner.hpp"
#include "core/visualization/overlays.hpp"
#include "motion_controller.hpp"
#include "motion_intent.hpp"

namespace ateam_kenobi::motion
{

class MotionExecutor
{
public:
  MotionExecutor(rclcpp::Logger logger);

  std::array<std::optional<BodyVelocity>,
    16> RunFrame(
    std::array<std::optional<MotionIntent>, 16> intents,
    visualization::Overlays & overlays, const World & world);

private:
  rclcpp::Logger logger_;
  std::array<path_planning::PathPlanner, 16> planners_;
  std::array<MotionController, 16> controllers_;

  std::optional<BodyVelocity> GenerateEscapeVelocity(
    const World & world, const Robot & robot,
    const MotionIntent & intent);

  void DrawOverlays(
    visualization::Overlays & overlays, const World & world,
    const Robot & robot, const path_planning::Path & path,
    const MotionIntent & intent);

  std::pair<size_t, ateam_geometry::Point> ProjectRobotOnPath(
    const path_planning::Path & path, const Robot & robot);

};

} // namespace ateam_kenobi::motion

#endif  // CORE__MOTION__MOTION_EXECUTOR_HPP_
