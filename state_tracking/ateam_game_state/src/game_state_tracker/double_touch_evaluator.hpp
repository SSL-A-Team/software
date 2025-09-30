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


#ifndef GAME_STATE_TRACKER__DOUBLE_TOUCH_EVAL_HPP_
#define GAME_STATE_TRACKER__DOUBLE_TOUCH_EVAL_HPP_

#include <algorithm>
#include <optional>
#include <vector>
#include <ateam_common/robot_constants.hpp>
#include "ateam_game_state/world.hpp"

namespace ateam_game_state
{

class DoubleTouchEvaluator
{
public:
  void Update(World & world);

private:
  static constexpr double kStartTouchBallsenseThreshold = kRobotRadius + 0.1;
  static constexpr double kStartTouchVisionThreshold = kRobotRadius + kBallRadius + 0.01;
  static constexpr double kEndTouchVisionThreshold = kRobotRadius + kBallRadius + 0.045;
  bool double_touch_rule_applies_ = false;
  std::optional<int> forbidden_id_;
  // tracking the frame when command changes, different from RefereeInfo.prev_command
  ateam_common::GameCommand prev_game_command_{ateam_common::GameCommand::Halt};
  std::optional<int> prev_touching_id_;

  std::optional<Robot> GetRobotTouchingBall(const World & world);
};

}  // namespace ateam_game_state

#endif  // GAME_STATE_TRACKER__DOUBLE_TOUCH_EVAL_HPP_
