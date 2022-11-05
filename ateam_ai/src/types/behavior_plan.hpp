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

#ifndef TYPES__BEHAVIOR_PLAN_HPP_
#define TYPES__BEHAVIOR_PLAN_HPP_

#include <optional>
#include <vector>

#include "types/trajectory.hpp"

struct KickFeedback {};
struct ReceiveFeedback {};
struct ShotFeedback {};
struct ReceiveShotFeedback {};
struct MoveFeedback {};
struct CostFeedback {};

struct BehaviorPlan
{
  std::optional<int> assigned_robot_id;
  Trajectory trajectory;
  DribblerAndKickerBehavior dribbler_and_kicker_behavior;

  using SpecificFeedback = std::variant<KickFeedback, ReceiveFeedback, ShotFeedback,
      ReceiveFeedback, MoveFeedback, CostFeedback>;
  std::variant<SpecificFeedback> specific_feedback;
};

#endif  // TYPES__BEHAVIOR_PLAN_HPP_
