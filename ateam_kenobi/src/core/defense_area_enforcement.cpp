// Copyright 2024 A Team
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

#include "defense_area_enforcement.hpp"
#include <ateam_geometry/ateam_geometry.hpp>
#include <ateam_common/robot_constants.hpp>
#include <ateam_controls_cpp/exceptions.hpp>
#include <ateam_controls_cpp/predict.hpp>
#include "core/motion/frame_conversions.hpp"

namespace ateam_kenobi::defense_area_enforcement
{

void EnforceDefenseAreaKeepout(
  const World & world,
  std::array<std::optional<motion::MotionCommand>, 16> & motion_commands,
  visualization::Overlays & overlays)
{
  if (IsDefenseAreaNavigationAllowed(world.referee_info.running_command)) {
    return;
  }

  const ateam_geometry::Rectangle our_defense_area{
    -((world.field.field_length / 2.0) + ( 2 * world.field.boundary_width ) +
    world.field.defense_area_depth ),
    -(world.field.defense_area_width / 2.0),
    -((world.field.field_length / 2.0) - world.field.defense_area_depth),
    world.field.defense_area_width / 2.0
  };
  const ateam_geometry::Rectangle their_defense_area {
    ((world.field.field_length / 2.0) + ( 2 * world.field.boundary_width ) +
    world.field.defense_area_depth ),
    -(world.field.defense_area_width / 2.0),
    ((world.field.field_length / 2.0) - world.field.defense_area_depth),
    world.field.defense_area_width / 2.0
  };
  motion::MotionCommand stop_command;
  stop_command.control_mode = motion::ControlMode::LocalVelocity;
  stop_command.velocity = {
    .x = 0.0,
    .y = 0.0,
    .theta = 0.0
  };

  for (auto robot_id = 0ul; robot_id < motion_commands.size(); ++robot_id) {
    if (robot_id == static_cast<std::size_t>(world.referee_info.our_goalie_id)) {
      continue;
    }
    const auto & robot = world.our_robots[robot_id];
    if(!robot.visible) {
      continue;
    }
    auto & maybe_command = motion_commands[robot_id];
    if (!maybe_command) {
      continue;
    }
    auto & command = *maybe_command;
    const auto new_pos = GetPredictedPosition(robot, command);
    if(IsRobotEscapingDefenseArea(robot.pos, new_pos, our_defense_area) ||
      IsRobotEscapingDefenseArea(robot.pos, new_pos, their_defense_area) ||
      IsRobotDestinationOutsideDefenseArea(command, our_defense_area) ||
      IsRobotDestinationOutsideDefenseArea(command, their_defense_area))
    {
      continue;
    }
    if(IsRobotGoingDeeperIntoDefenseArea(robot.pos, new_pos, our_defense_area) ||
      IsRobotGoingDeeperIntoDefenseArea(robot.pos, new_pos,
        their_defense_area))
    {
      command = stop_command;
      overlays.drawStopsign("defkeepout/robot" + std::to_string(robot.id), robot, "Red");
    }
  }
}

ateam_geometry::Point GetPredictedPosition(
  const Robot & robot,
  const motion::MotionCommand & command)
{
  try {
    constexpr double lookahead = 0.5;
    using namespace ateam_controls_cpp::predict;  // NOLINT(build/namespaces)
    switch(command.control_mode) {
      case motion::ControlMode::Off:
        return PositionAtT(robot, modes::Off{}, lookahead);
      case motion::ControlMode::EStopBrake:
        return PositionAtT(robot, modes::EStopBrake{}, lookahead);
      case motion::ControlMode::GlobalPosition:
        return PositionAtT(robot, modes::GlobalPosition{
          .global_x = static_cast<float>(command.pose.x),
          .global_y = static_cast<float>(command.pose.y),
          .global_theta = static_cast<float>(command.pose.theta),
          .max_linear_vel = static_cast<float>(command.limit_vel_linear),
          .max_angular_vel = static_cast<float>(command.limit_vel_angular),
          .max_linear_acc = static_cast<float>(command.limit_acc_linear),
          .max_angular_acc = static_cast<float>(command.limit_acc_angular)
        }, lookahead);
      case motion::ControlMode::GlobalVelocity:
        return PositionAtT(robot, modes::GlobalVelocity{}, lookahead);
      case motion::ControlMode::LocalVelocity:
        return PositionAtT(robot, modes::LocalVelocity{}, lookahead);
      case motion::ControlMode::GlobalAccel:
        return PositionAtT(robot, modes::GlobalAccel{}, lookahead);
      case motion::ControlMode::LocalAccel:
        return PositionAtT(robot, modes::LocalAccel{}, lookahead);
      case motion::ControlMode::HeadingPivot:
        return PositionAtT(robot, modes::HeadingPivot{}, lookahead);
      case motion::ControlMode::PointPivot:
        return PositionAtT(robot, modes::PointPivot{}, lookahead);
      case motion::ControlMode::HeadingLine:
        return PositionAtT(robot, modes::HeadingLine{}, lookahead);
      case motion::ControlMode::PointLine:
        return PositionAtT(robot, modes::PointLine{}, lookahead);
      default:
        return robot.pos;
    }
  } catch(const ateam_controls_cpp::ControlsException &) {
    return robot.pos;
  }
}

bool IsRobotGoingDeeperIntoDefenseArea(
  const ateam_geometry::Point & position,
  const ateam_geometry::Point & new_position,
  const ateam_geometry::Rectangle & defense_area)
{
  const auto robot_footprint = ateam_geometry::makeDisk(new_position, kRobotRadius);
  if(!CGAL::do_intersect(robot_footprint, defense_area)) {
    return false;
  }
  const auto area_center = CGAL::midpoint(defense_area.min(), defense_area.max());
  return CGAL::compare_distance_to_point(area_center, position, new_position) == CGAL::LARGER;
}

bool IsRobotEscapingDefenseArea(
  const ateam_geometry::Point & position,
  const ateam_geometry::Point & new_position,
  const ateam_geometry::Rectangle & defense_area)
{
  const auto robot_footprint = ateam_geometry::makeDisk(position, kRobotRadius);
  if(!CGAL::do_intersect(robot_footprint, defense_area)) {
    return false;
  }
  const auto area_center = CGAL::midpoint(defense_area.min(), defense_area.max());
  return CGAL::compare_distance_to_point(area_center, position, new_position) == CGAL::SMALLER;
}

bool IsRobotDestinationOutsideDefenseArea(
  const motion::MotionCommand & command,
  const ateam_geometry::Rectangle & defense_area)
{
  if(command.control_mode == motion::ControlMode::GlobalPosition) {
    const ateam_geometry::Point position{command.pose.x, command.pose.y};
    const auto robot_footprint = ateam_geometry::makeDisk(position, kRobotRadius);
    return CGAL::do_intersect(robot_footprint, defense_area);
  }
  return false;
}

bool IsDefenseAreaNavigationAllowed(const ateam_common::GameCommand & command)
{
  return command == ateam_common::GameCommand::BallPlacementOurs ||
         command == ateam_common::GameCommand::BallPlacementTheirs;
}

}  // namespace ateam_kenobi::defense_area_enforcement
