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

#ifndef CORE__DEFENSE_AREA_ENFORCEMENT_HPP_
#define CORE__DEFENSE_AREA_ENFORCEMENT_HPP_

#include <ateam_msgs/msg/robot_motion_command.hpp>
#include "core/types/state_types.hpp"

namespace ateam_kenobi::defense_area_enforcement
{

/**
 * @brief Prevents sending motion commands that would encroach on defense areaas
 *
 * Any velocity command that would lead to a collision with a defense area will be set to zero.
 *
 * @note Goalie is ommitted from enforcement.
 */
void EnforceDefenseAreaKeepout(
  const World & world,
  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
  16> & motion_commands);

bool WouldVelocityCauseCollision(
  const World & world, const int robot_id,
  const ateam_msgs::msg::RobotMotionCommand & motion_command);

bool IsRobotEscapingDefenseArea(
  const ateam_geometry::Point & position,
  const ateam_geometry::Point & new_position,
  const ateam_geometry::Rectangle & defense_area);

bool IsDefenseAreaNavigationAllowed(const ateam_common::GameCommand & command);

}  // namespace ateam_kenobi::defense_area_enforcement


#endif  // CORE__DEFENSE_AREA_ENFORCEMENT_HPP_
