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

#include "motion_executor.hpp"
#include <utility>
#include <vector>
#include <ateam_geometry/nearest_point.hpp>
#include "core/path_planning/obstacles.hpp"
#include "core/path_planning/escape_velocity.hpp"
#include "frame_conversions.hpp"
#include "pivot_control.hpp"

namespace ateam_kenobi::motion
{

MotionExecutor::MotionExecutor(rclcpp::Logger logger)
: logger_(std::move(logger))
{
}

std::array<std::optional<MotionCommand>,
  16> MotionExecutor::RunFrame(
  std::array<std::optional<MotionIntent>, 16> intents,
  visualization::Overlays & overlays, const World & world)
{
  std::array<std::optional<MotionCommand>, 16> commands{};

  for(auto i = 0ul; i < 16; ++i) {
    const auto & robot = world.our_robots[i];
    const auto & maybe_intent = intents[i];
    if(!maybe_intent.has_value()) {
      continue;
    }
    const auto & intent = *maybe_intent;
    std::visit([this, &robot, &overlays, &world, &commands, &i](const auto & intent){
        commands[i] = ExecuteIntent(intent, robot, overlays, world);
    }, intent);
  }

  ExecutePathPlanningTargets(commands, overlays, world);

  return commands;
}

void MotionExecutor::ExecutePathPlanningTargets(
  std::array<std::optional<MotionCommand>, 16> & commands,
  visualization::Overlays & overlays, const World & world)
{
  (void)commands;
  (void)overlays;
  (void)world;
}

std::optional<MotionCommand> MotionExecutor::ExecuteIntent(
  const intents::None & intent, const Robot & robot, visualization::Overlays & overlays,
  const World & world)
{
  (void)robot;
  (void)intent;
  (void)overlays;
  (void)world;
  MotionCommand command;
  command.control_mode = ControlMode::Off;
  return command;
}

std::optional<MotionCommand> MotionExecutor::ExecuteIntent(
  const intents::Stop & intent, const Robot & robot, visualization::Overlays & overlays,
  const World & world)
{
  (void)robot;
  (void)overlays;
  (void)world;
  MotionCommand command;
  command.control_mode = ControlMode::LocalVelocity;
  command.velocity.x = 0.0;
  command.velocity.y = 0.0;
  command.velocity.theta = 0.0;
  command.limit_vel_linear = intent.limits.linear_velocity;
  command.limit_vel_angular = intent.limits.angular_velocity;
  command.limit_acc_linear = intent.limits.linear_acceleration;
  command.limit_acc_angular = intent.limits.angular_acceleration;
  return command;
}

std::optional<MotionCommand> MotionExecutor::ExecuteIntent(
  const intents::Velocity & intent, const Robot & robot, visualization::Overlays & overlays,
  const World & world)
{
  (void)robot;
  (void)overlays;
  (void)world;
  MotionCommand command;
  switch(intent.frame) {
    case Frame::Local:
      command.control_mode = ControlMode::LocalVelocity;
      break;
    case Frame::World:
      command.control_mode = ControlMode::GlobalVelocity;
      break;
  }
  command.velocity.x = intent.linear.x();
  command.velocity.y = intent.linear.y();
  command.velocity.theta = intent.angular;
  command.limit_vel_linear = intent.limits.linear_velocity;
  command.limit_vel_angular = intent.limits.angular_velocity;
  command.limit_acc_linear = intent.limits.linear_acceleration;
  command.limit_acc_angular = intent.limits.angular_acceleration;
  return command;
}

std::optional<MotionCommand> MotionExecutor::ExecuteIntent(
  const intents::Position & intent, const Robot & robot, visualization::Overlays & overlays,
  const World & world)
{
  (void)overlays;
  (void)world;
  path_planning_targets_.push_back(PathPlanningTarget{
      robot.id,
      intent.position,
      intent.heading,
      intent.planner_options,
      intent.obstacles,
      intent.enable_escape_velocities,
      intent.limits
  });
  return std::nullopt;
}

std::optional<MotionCommand> MotionExecutor::ExecuteIntent(
  const intents::PositionFacing & intent, const Robot & robot, visualization::Overlays & overlays,
  const World & world)
{
  (void)overlays;
  (void)world;
  const auto heading = 0;  // TODO get heading to target
  path_planning_targets_.push_back(PathPlanningTarget{
      robot.id,
      intent.position,
      heading,
      intent.planner_options,
      intent.obstacles,
      intent.enable_escape_velocities,
      intent.limits
  });
  return std::nullopt;
}

std::optional<MotionCommand> MotionExecutor::ExecuteIntent(
  const intents::PivotVelocity & intent, const Robot & robot, visualization::Overlays & overlays,
  const World & world)
{
  (void)robot;
  (void)overlays;
  (void)world;
  return PivotAtVelocity(intent);
}

std::optional<MotionCommand> MotionExecutor::ExecuteIntent(
  const intents::PivotHeading & intent, const Robot & robot, visualization::Overlays & overlays,
  const World & world)
{
  (void)overlays;
  (void)world;
  return PivotToHeading(intent, robot);
}

}  // namespace ateam_kenobi::motion
