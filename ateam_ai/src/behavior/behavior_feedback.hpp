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

#ifndef BEHAVIOR__BEHAVIOR_FEEDBACK_HPP_
#define BEHAVIOR__BEHAVIOR_FEEDBACK_HPP_

#include <Eigen/Dense>

struct Trajectory
{
  // TODO(jneiger): Flesh out the trajectory structure more based on Kyle's requirements
  Eigen::Vector2d target_point;
};
struct DribblerAndKickerBehavior {};

struct BehaviorFeedback
{
  std::optional<int> assigned_robot_id;
  // behavior start time
  // behavior end time
  // behavior wasted time (how long does this robot need to
  //   wait for other robots to get into position)
  Trajectory trajectory;
  DribblerAndKickerBehavior dribbler_and_kicker_behavior;
};

#endif  // BEHAVIOR__BEHAVIOR_FEEDBACK_HPP_
