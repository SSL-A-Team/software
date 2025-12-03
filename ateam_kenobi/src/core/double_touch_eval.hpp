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


#ifndef CORE__DOUBLE_TOUCH_EVAL_HPP_
#define CORE__DOUBLE_TOUCH_EVAL_HPP_

#include <algorithm>
#include <vector>
#include <ateam_common/robot_constants.hpp>
#include <boost/scope_exit.hpp>
#include "core/types/state_types.hpp"
#include "core/visualization/overlays.hpp"

namespace ateam_kenobi
{

class DoubleTouchEval
{
public:
  void update(World & world, visualization::Overlays & overlays)
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

    DrawForbiddenBotOverlay(world, overlays);
  }

private:
  static constexpr double kStartTouchBallsenseThreshold = kRobotRadius + 0.1;
  static constexpr double kStartTouchVisionThreshold = kRobotRadius + kBallRadius + 0.01;
  static constexpr double kEndTouchVisionThreshold = kRobotRadius + kBallRadius + 0.045;
  bool double_touch_rule_applies_ = false;
  std::optional<int> forbidden_id_;
  // tracking the frame when command changes, different from RefereeInfo.prev_command
  ateam_common::GameCommand prev_game_command_{ateam_common::GameCommand::Halt};
  std::optional<int> prev_touching_id_;

  std::optional<Robot> GetRobotTouchingBall(const World & world)
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

  void DrawForbiddenBotOverlay(const World & world, visualization::Overlays & overlays)
  {
    if(!world.double_touch_forbidden_id_) {
      return;
    }
    const auto forbidden_id = world.double_touch_forbidden_id_.value();
    const auto & forbidden_bot = world.our_robots[forbidden_id];
    const auto & bot_pos = forbidden_bot.pos;
    const auto side_len = kRobotDiameter;
    const auto small_dim = side_len / 2.0;
    const auto big_dim = small_dim + (side_len * std::cos(M_PI / 4.0));
    std::vector<ateam_geometry::Point> points {
      bot_pos + ateam_geometry::Vector{big_dim, small_dim},
      bot_pos + ateam_geometry::Vector{small_dim, big_dim},
      bot_pos + ateam_geometry::Vector{-small_dim, big_dim},
      bot_pos + ateam_geometry::Vector{-big_dim, small_dim},
      bot_pos + ateam_geometry::Vector{-big_dim, -small_dim},
      bot_pos + ateam_geometry::Vector{-small_dim, -big_dim},
      bot_pos + ateam_geometry::Vector{small_dim, -big_dim},
      bot_pos + ateam_geometry::Vector{big_dim, -small_dim}
    };
    ateam_geometry::Polygon octagon{points.begin(), points.end()};
    overlays.drawPolygon("double_touch_forbidden_bot", octagon, "DarkRed", "#00000000");
  }
};

}  // namespace ateam_kenobi

#endif  // CORE__DOUBLE_TOUCH_EVAL_HPP_
