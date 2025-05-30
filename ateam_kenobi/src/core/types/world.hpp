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


#ifndef CORE__TYPES__WORLD_HPP_
#define CORE__TYPES__WORLD_HPP_

#include <optional>
#include <array>
#include <chrono>
#include <ateam_spatial/spatial_evaluator.hpp>

#include "core/types/ball.hpp"
#include "core/types/field.hpp"
#include "core/types/referee_info.hpp"
#include "core/types/robot.hpp"

namespace ateam_kenobi
{
struct World
{
  std::chrono::steady_clock::time_point current_time;

  Field field;
  RefereeInfo referee_info;

  Ball ball;
  std::array<Robot, 16> our_robots;
  std::array<Robot, 16> their_robots;

  bool in_play = false;
  bool our_penalty = false;
  bool their_penalty = false;

  int ignore_side = 0;

  // Holds the ID of the robot not allowed to touch the ball, if any
  std::optional<int> double_touch_forbidden_id_;

  ateam_spatial::SpatialEvaluator * spatial_evaluator;
};
}  // namespace ateam_kenobi

#endif  // CORE__TYPES__WORLD_HPP_
