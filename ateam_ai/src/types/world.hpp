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


#ifndef TYPES__WORLD_HPP_
#define TYPES__WORLD_HPP_

#include <optional>
#include <array>
#include <vector>

#include "types/ball.hpp"
#include "types/field.hpp"
#include "types/referee_info.hpp"
#include "types/robot.hpp"
#include "types/trajectory.hpp"

struct BehaviorExecutorState
{
  // Trajectory of each robot last frame
  std::array<std::optional<Trajectory>, 16> previous_trajectories;
};

struct World
{
  const double immutable_duration = 0.1;  // s
  double current_time = 0.0;  // s

  Field field;
  RefereeInfo referee_info;

  std::optional<Ball> get_unique_ball() const
  {
    if (balls.size() == 1) {
      return balls.front();
    } else {
      return std::nullopt;
    }
  }

  std::vector<Ball> balls;
  std::array<std::optional<Robot>, 16> our_robots;
  std::array<std::optional<Robot>, 16> their_robots;

  std::array<std::optional<Robot>, 16> plan_from_our_robots;

  BehaviorExecutorState behavior_executor_state;
};

#endif  // TYPES__WORLD_HPP_
