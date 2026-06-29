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
#include <ateam_geometry/angles.hpp>
#include <ateam_geometry/nearest_point.hpp>
#include "path_planning/obstacles.hpp"
#include "escape_velocity.hpp"
#include "frame_conversions.hpp"
#include "pivot_control.hpp"

namespace ateam_kenobi::motion
{

MotionExecutor::MotionExecutor(rclcpp::Logger logger)
: logger_(std::move(logger))
{
  // TODO(barulim): Tune heading PID
  std::fill(heading_controllers_.begin(), heading_controllers_.end(), PID{1.0, 0.0, 0.0});
}

std::array<std::optional<MotionCommand>,
  16> MotionExecutor::RunFrame(
  std::array<std::optional<MotionIntent>, 16> intents,
  visualization::Overlays & overlays, const World & world)
{
  std::array<std::optional<MotionCommand>, 16> commands{};

  path_planning_targets_.clear();

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

  planner_.Execute(commands, path_planning_targets_, world, overlays);

  return commands;
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
  const intents::LinearVelocityAngularHeading & intent, const Robot & robot,
  visualization::Overlays & overlays, const World & world)
{
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
  auto angular_controller = heading_controllers_[robot.id];
  command.velocity.theta =
    angular_controller.compute_command(angles::shortest_angular_distance(intent.heading,
      robot.theta), 0.01);
  command.limit_vel_linear = intent.limits.linear_velocity;
  command.limit_vel_angular = intent.limits.angular_velocity;
  command.limit_acc_linear = intent.limits.linear_acceleration;
  command.limit_acc_angular = intent.limits.angular_acceleration;
  return command;
}

std::optional<MotionCommand> MotionExecutor::ExecuteIntent(
  const intents::LinearVelocityAngularFacing & intent, const Robot & robot,
  visualization::Overlays & overlays, const World & world)
{
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
  auto angular_controller = heading_controllers_[robot.id];
  const auto target_heading = ateam_geometry::ToHeading(intent.face_target - robot.pos);
  command.velocity.theta =
    angular_controller.compute_command(angles::shortest_angular_distance(target_heading,
      robot.theta), 0.01);
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
  path_planning_targets_.push_back(path_planning::PathPlanningTarget{
      robot.id,
      intent.position,
      intent.heading,
      intent.planner_options,
      intent.obstacles,
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
  const auto heading = atan2(
          intent.face_target.y() - robot.pos.y(),
          intent.face_target.x() - robot.pos.x());
  path_planning_targets_.push_back(path_planning::PathPlanningTarget{
      robot.id,
      intent.position,
      heading,
      intent.planner_options,
      intent.obstacles,
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
  (void)robot;

  MotionCommand command;
  command.control_mode = ControlMode::HeadingPivot;
  command.limit_vel_angular = intent.limits.angular_velocity;
  command.limit_acc_angular = intent.limits.angular_acceleration;
  command.pivot_global_theta = intent.target_heading;
  command.pivot_orbit_radius = intent.radius;
  command.pivot_inset_angle = intent.inset_angle;
  command.pivot_direction = static_cast<uint8_t>(intent.direction);
  command.pivot_commpute_inset_angle = intent.compute_inset_angle;

  return command;
}

std::optional<MotionCommand> MotionExecutor::ExecuteIntent(
  const intents::PivotPoint & intent, const Robot & robot, visualization::Overlays & overlays,
  const World & world)
{
  (void)overlays;
  (void)world;
  (void)robot;

  MotionCommand command;
  command.control_mode = ControlMode::PointPivot;
  command.limit_vel_angular = intent.limits.angular_velocity;
  command.limit_acc_angular = intent.limits.angular_acceleration;
  command.pivot_target_x = intent.target_x;
  command.pivot_target_y = intent.target_y;
  command.pivot_orbit_radius = intent.radius;
  command.pivot_inset_angle = intent.inset_angle;
  command.pivot_direction = static_cast<uint8_t>(intent.direction);
  command.pivot_commpute_inset_angle = intent.compute_inset_angle;

  return command;
}

}  // namespace ateam_kenobi::motion
