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

#ifndef CORE__MOTION__MOTION_INTENT_HPP_
#define CORE__MOTION__MOTION_INTENT_HPP_

#include <functional>
#include <optional>
#include <variant>
#include <vector>
#include <ateam_geometry/types.hpp>
#include "core/types/state_types.hpp"
#include "motion_options.hpp"
#include "path_planning/planner_options.hpp"

namespace ateam_kenobi::motion
{

template<typename T>
concept has_planner_options = requires(T intent) {
  intent.planner_options;
};

template<typename T>
concept has_limits = requires(T intent) {
  intent.limits;
};

enum class Frame
{
  World,
  Local
};

enum class PivotDirection
{
  Forward = 0,
  Backward = 1
};

struct Limits
{
  double linear_velocity = 0.0;
  double linear_acceleration = 0.0;
  double angular_velocity = 0.0;
  double angular_acceleration = 0.0;
};

namespace intents
{

struct None {};

struct Stop
{
  Limits limits;
};

struct Velocity
{
  ateam_geometry::Vector linear;
  double angular;
  Frame frame = Frame::World;
  Limits limits;
};

struct LinearVelocityAngularHeading
{
  ateam_geometry::Vector linear;
  double heading;
  Frame frame = Frame::World;
  Limits limits;
};

struct LinearVelocityAngularFacing
{
  ateam_geometry::Vector linear;
  ateam_geometry::Point face_target;
  Frame frame = Frame::World;
  Limits limits;
};

struct Position
{
  ateam_geometry::Point position;
  double heading;
  path_planning::PlannerOptions planner_options;
  std::vector<ateam_geometry::AnyShape> obstacles;
  Limits limits;
};

struct PositionFacing
{
  ateam_geometry::Point position;
  ateam_geometry::Point face_target;
  path_planning::PlannerOptions planner_options;
  std::vector<ateam_geometry::AnyShape> obstacles;
  Limits limits;
};

struct PivotVelocity
{
  double angular_velocity;
  double radius = 0.089;  // Estimated radius of bot holding ball
  Limits limits;
};

struct PivotHeading
{
  double target_heading;
  double radius = 0.089;  // Estimated radius of bot holding ball
  double inset_angle = 0.0;
  PivotDirection direction = PivotDirection::Forward;
  bool compute_inset_angle = false;
  Limits limits;
};

struct PivotPoint
{
  double target_x;
  double target_y;
  double radius = 0.089;  // Estimated radius of bot holding ball
  double inset_angle = 0.0;
  PivotDirection direction = PivotDirection::Forward;
  bool compute_inset_angle = false;
  Limits limits;
};

}  // namespace intents

using MotionIntent = std::variant<
  intents::None,
  intents::Stop,
  intents::Velocity,
  intents::LinearVelocityAngularHeading,
  intents::LinearVelocityAngularFacing,
  intents::Position,
  intents::PositionFacing,
  intents::PivotVelocity,
  intents::PivotHeading,
  intents::PivotPoint>;

}  // namespace ateam_kenobi::motion

#endif  // CORE__MOTION__MOTION_INTENT_HPP_
