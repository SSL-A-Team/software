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

#include "trajectory_generation/trajectory_generation.hpp"

BehaviorFeedback TrajectoryGenerator::get_feedback_from_behavior(
  Behavior behavior /**, Robot assigned_robot**/)
{
  BehaviorFeedback feedback;

  switch (behavior.type) {
    case Behavior::Type::MovingKick:
    case Behavior::Type::PivotKick:
      feedback.trajectory.target_point = std::get<KickParam>(behavior.params).target_location;
      break;

    case Behavior::Type::OneTouchReceiveKick:
    case Behavior::Type::TwoTouchReceiveKick:
      feedback.trajectory.target_point = std::get<ReceiveParam>(behavior.params).receive_location;
      break;

    case Behavior::Type::Shot:
      feedback.trajectory.target_point = Eigen::Vector2d{0, 0};
      break;

    case Behavior::Type::OneTouchShot:
      feedback.trajectory.target_point =
        std::get<ReceiveShotParam>(behavior.params).receive_location;
      break;

    case Behavior::Type::MoveToPoint:
      feedback.trajectory.target_point = std::get<MoveParam>(behavior.params).target_location;
      break;

    case Behavior::Type::CostFunctionPoint:
      feedback.trajectory.target_point = Eigen::Vector2d{0, 0};
      break;

    default:
      feedback.trajectory.target_point = Eigen::Vector2d{0, 0};
      break;
  }

  return feedback;
}
