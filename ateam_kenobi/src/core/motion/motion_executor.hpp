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
#include <utility>
#include <vector>
#include <rclcpp/logger.hpp>
#include "path_planning/path_planner.hpp"
#include "path_planning/path_planning_target.hpp"
#include "core/visualization/overlays.hpp"
#include "motion_intent.hpp"
#include "motion_command.hpp"
#include "pid.hpp"

namespace ateam_kenobi::motion
{

class MotionExecutor
{
public:
  explicit MotionExecutor(rclcpp::Logger logger);

  std::array<std::optional<MotionCommand>,
    16> RunFrame(
    std::array<std::optional<MotionIntent>, 16> intents,
    visualization::Overlays & overlays, const World & world);

private:
  rclcpp::Logger logger_;
  path_planning::PathPlanner planner_;
  std::array<PID, 16> heading_controllers_;
  std::vector<path_planning::PathPlanningTarget> path_planning_targets_;

  std::optional<MotionCommand> ExecuteIntent(
    const intents::None & intent, const Robot & robot,
    visualization::Overlays & overlays, const World & world);
  std::optional<MotionCommand> ExecuteIntent(
    const intents::Stop & intent, const Robot & robot,
    visualization::Overlays & overlays, const World & world);
  std::optional<MotionCommand> ExecuteIntent(
    const intents::Velocity & intent, const Robot & robot,
    visualization::Overlays & overlays, const World & world);
  std::optional<MotionCommand> ExecuteIntent(
    const intents::LinearVelocityAngularHeading & intent, const Robot & robot,
    visualization::Overlays & overlays, const World & world);
  std::optional<MotionCommand> ExecuteIntent(
    const intents::LinearVelocityAngularFacing & intent, const Robot & robot,
    visualization::Overlays & overlays, const World & world);
  std::optional<MotionCommand> ExecuteIntent(
    const intents::Position & intent, const Robot & robot,
    visualization::Overlays & overlays, const World & world);
  std::optional<MotionCommand> ExecuteIntent(
    const intents::PositionFacing & intent, const Robot & robot,
    visualization::Overlays & overlays, const World & world);
  std::optional<MotionCommand> ExecuteIntent(
    const intents::PivotVelocity & intent, const Robot & robot,
    visualization::Overlays & overlays, const World & world);
  std::optional<MotionCommand> ExecuteIntent(
    const intents::PivotHeading & intent, const Robot & robot,
    visualization::Overlays & overlays, const World & world);
  std::optional<MotionCommand> ExecuteIntent(
    const intents::PivotPoint & intent, const Robot & robot,
    visualization::Overlays & overlays, const World & world);
  std::optional<MotionCommand> ExecuteIntent(
    const intents::LineHeading & intent, const Robot & robot,
    visualization::Overlays & overlays, const World & world);
  std::optional<MotionCommand> ExecuteIntent(
    const intents::LinePoint & intent, const Robot & robot,
    visualization::Overlays & overlays, const World & world);
};

}  // namespace ateam_kenobi::motion

#endif  // CORE__MOTION__MOTION_EXECUTOR_HPP_
