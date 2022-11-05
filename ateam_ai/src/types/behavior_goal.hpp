// Copyright 2021 A Team
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

#ifndef TYPES__BEHAVIOR_GOAL_HPP_
#define TYPES__BEHAVIOR_GOAL_HPP_

#include <Eigen/Dense>

#include <optional>
#include <variant>
#include <functional>

struct KickParam
{
  Eigen::Vector2d target_location;

  explicit KickParam(Eigen::Vector2d target_location)
  : target_location(target_location) {}
};

struct ReceiveParam
{
  Eigen::Vector2d receive_location;
  Eigen::Vector2d target_location;

  ReceiveParam(Eigen::Vector2d receive_location, Eigen::Vector2d target_location)
  : receive_location(receive_location), target_location(target_location) {}
};

struct ShotParam {};

struct ReceiveShotParam
{
  Eigen::Vector2d receive_location;

  explicit ReceiveShotParam(Eigen::Vector2d receive_location)
  : receive_location(receive_location) {}
};

struct MoveParam
{
  Eigen::Vector2d target_location;

  explicit MoveParam(Eigen::Vector2d target_location)
  : target_location(target_location) {}
};

struct CostParam
{
  std::function<double(Eigen::Vector2d)> cost;
  std::optional<std::function<Eigen::Vector2d(Eigen::Vector2d)>> gradient;
};

struct BehaviorGoal
{
  enum Type
  {
    // Straight kick
    MovingKick,
    PivotKick,

    // Receive + kick
    OneTouchReceiveKick,
    TwoTouchReceiveKick,

    // Kick + instantaneous target selection near kick time
    Shot,
    OneTouchShot,

    // Normal moves
    MoveToPoint,
    CostFunctionPoint
  } type;

  enum Priority
  {
    Required,
    Medium,
    Low
  } priority;

  using Params = std::variant<KickParam, ReceiveParam, ShotParam, ReceiveShotParam, MoveParam,
      CostParam>;
  Params params;

  // TODO(jneiger): Add type<->param checking for consistency
  BehaviorGoal(Type type, Priority priority, Params params)
  : type(type), priority(priority), params(params) {}
};

#endif  // TYPES__BEHAVIOR_GOAL_HPP_
