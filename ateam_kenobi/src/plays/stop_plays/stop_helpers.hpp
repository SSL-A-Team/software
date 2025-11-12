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

#ifndef PLAYS__STOP_PLAYS__STOP_HELPERS_HPP_
#define PLAYS__STOP_PLAYS__STOP_HELPERS_HPP_

#include <vector>
#include <optional>
#include <array>
#include <rclcpp/rclcpp.hpp>
#include <nlohmann/json.hpp>
#include <ateam_common/robot_constants.hpp>
#include <ateam_geometry/types.hpp>
#include "core/types/state_types.hpp"
#include "core/types/robot_command.hpp"
#include "core/visualization/overlays.hpp"

namespace ateam_kenobi::plays::stop_plays::stop_helpers
{

static constexpr double kKeepoutRadiusRules = 0.5;
static constexpr double kKeepoutRadius = kKeepoutRadiusRules + kRobotRadius + 0.2;

std::vector<ateam_geometry::Point> getOpenSpots(
  const World & world,
  visualization::Overlays & overlays);

void removeArc(std::vector<ateam_geometry::Arc> & openings, const ateam_geometry::Arc & arc);

bool isPointInOrBehindGoal(const ateam_geometry::Point & point, const World & world);

std::vector<ateam_geometry::AnyShape> getAddedObstacles(const World & world);

void drawObstacles(
  const World & world,
  const std::vector<ateam_geometry::AnyShape> & added_obstacles, visualization::Overlays & overlays,
  rclcpp::Logger & logger);

void moveBotsTooCloseToBall(
  const World & world,
  const std::vector<ateam_geometry::AnyShape> & added_obstacles,
  std::array<std::optional<RobotCommand>, 16> & motion_commands,
  visualization::Overlays & overlays,
  nlohmann::json & play_info);

void moveBotsInObstacles(
  const World & world,
  const std::vector<ateam_geometry::AnyShape> & added_obstacles,
  std::array<std::optional<RobotCommand>, 16> & motion_commands,
  nlohmann::json & play_info);

}  // namespace ateam_kenobi::plays::stop_plays::stop_helpers

#endif  // PLAYS__STOP_PLAYS__STOP_HELPERS_HPP_
