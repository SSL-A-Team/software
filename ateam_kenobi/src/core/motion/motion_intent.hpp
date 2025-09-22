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

#ifndef CORE__MOTION__MOTION_TARGET_HPP_
#define CORE__MOTION__MOTION_TARGET_HPP_

#include <functional>
#include <optional>
#include <variant>
#include <ateam_geometry/types.hpp>
#include "core/types/world.hpp"
#include "core/types/robot.hpp"
#include "core/path_planning/path.hpp"
#include "core/path_planning/planner_options.hpp"
#include "motion_options.hpp"

namespace ateam_kenobi::motion
{

namespace intents
{

struct None {};

namespace linear
{

enum class Frame
{
  World,
  Local
};

struct VelocityIntent
{
  ateam_geometry::Vector velocity;
  Frame frame = Frame::World;
};

struct PositionIntent
{
  ateam_geometry::Point position;
};

struct VelocityAtPositionIntent
{
  ateam_geometry::Point position;
  ateam_geometry::Vector velocity;  // Must be world frame
};

}  // namespace linear

namespace angular
{

struct VelocityIntent
{
  double omega;
};

struct HeadingIntent
{
  double theta;
};

struct FacingIntent
{
  ateam_geometry::Point target;
};

struct FaceTravelIntent {};

}  // namespace angular

}  // namespace intents

/// @brief Intended command velocity in local frame
struct BodyVelocity
{
  ateam_geometry::Vector linear{0.0, 0.0};
  double angular = 0.0;
};

struct MotionIntent
{
  using LinearIntent = std::variant<intents::None, intents::linear::VelocityIntent,
      intents::linear::PositionIntent,
      intents::linear::VelocityAtPositionIntent>;
  using AngularIntent = std::variant<intents::None, intents::angular::VelocityIntent,
      intents::angular::HeadingIntent,
      intents::angular::FacingIntent, intents::angular::FaceTravelIntent>;
  using PostCallback = std::function<BodyVelocity(BodyVelocity, const path_planning::Path &,
      const Robot &, const World &)>;

  LinearIntent linear = intents::None{};
  AngularIntent angular = intents::None{};
  std::optional<PostCallback> callback;
  path_planning::PlannerOptions planner_options;
  MotionOptions motion_options;
  std::vector<ateam_geometry::AnyShape> obstacles;
  bool enable_escape_velocities = true;
};

} // namespace ateam_kenobi::motion

#endif  // CORE__MOTION__MOTION_TARGET_HPP_
