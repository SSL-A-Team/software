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

#include "double_touch_evaluator.hpp"
#include <boost/scope_exit.hpp>

namespace ateam_game_state
{

void DoubleTouchEvaluator::Update(World & world)
{
  const auto running_command = world.referee_info.running_command;
  BOOST_SCOPE_EXIT(&running_command, this_) {
      this_->prev_game_command_ = running_command;
  } BOOST_SCOPE_EXIT_END

  if (running_command != ateam_common::GameCommand::NormalStart &&
    running_command != ateam_common::GameCommand::PrepareKickoffOurs &&
    running_command != ateam_common::GameCommand::DirectFreeOurs)
  {
    // Game is in a state where the double-touch rule does not apply.
    double_touch_rule_applies_ = false;
  }

  if (running_command != prev_game_command_ &&
    (running_command == ateam_common::GameCommand::PrepareKickoffOurs ||
    running_command == ateam_common::GameCommand::DirectFreeOurs))
  {
    // Entering state where double-touch rule applies
    forbidden_id_.reset();
    prev_touching_id_.reset();
    double_touch_rule_applies_ = true;
  }

  if (!double_touch_rule_applies_) {
    world.double_touch_forbidden_id_.reset();
    return;
  }

  const auto maybe_toucher = GetRobotTouchingBall(world);

  if (!prev_touching_id_ && maybe_toucher) {
    // A robot has started touching the ball
    const auto toucher_id = maybe_toucher.value().id;
    if (forbidden_id_ && toucher_id != forbidden_id_.value()) {
      // This is the second robot to touch the ball, double-touch hold is released
      forbidden_id_.reset();
      double_touch_rule_applies_ = false;
      world.double_touch_forbidden_id_.reset();
      return;
    }
  }

  if (prev_touching_id_ && (!maybe_toucher || world.in_play)) {
    // A robot has stopped touching the ball, or moved the ball enough to put it in play
    if (!forbidden_id_) {
      // This was the first robot to touch the ball, it can't touch it again
      forbidden_id_ = prev_touching_id_;
    }
  }

  world.double_touch_forbidden_id_ = forbidden_id_;

  if (!maybe_toucher) {
    prev_touching_id_.reset();
  } else {
    prev_touching_id_ = maybe_toucher.value().id;
  }
}

std::optional<Robot> DoubleTouchEvaluator::GetRobotTouchingBall(const World & world)
{
  auto found_iter = std::ranges::find_if(
      world.our_robots, [this, &world](const Robot & robot) {
      if (!robot.visible) {
        return false;
      }
      const auto ball_bot_distance = std::sqrt(
          CGAL::squared_distance(robot.pos, world.ball.pos));
      if (robot.breakbeam_ball_detected_filtered &&
      ball_bot_distance <= kStartTouchBallsenseThreshold)
      {
        return true;
      }
      if (prev_touching_id_.value_or(-1) == robot.id) {
        return ball_bot_distance <= kEndTouchVisionThreshold;
      } else {
        return ball_bot_distance <= kStartTouchVisionThreshold;
      }
      });
  if (found_iter == world.our_robots.end()) {
    return {};
  }
  return *found_iter;
}

}  // namespace ateam_game_state
