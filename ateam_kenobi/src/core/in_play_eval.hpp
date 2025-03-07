// Copyright 2024 A Team
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


#ifndef IN_PLAY_EVAL_HPP_
#define IN_PLAY_EVAL_HPP_

#include <chrono>
#include <optional>
#include <ateam_geometry/types.hpp>
#include <ateam_common/game_controller_listener.hpp>
#include "core/types/world.hpp"

namespace ateam_kenobi
{

class InPlayEval
{
public:
  void Update(World & world);

private:
  bool in_play_ = false;
  std::optional<ateam_geometry::Point> ball_start_pos_;
  ateam_common::GameCommand prev_game_command_;
  std::optional<std::chrono::steady_clock::duration> timeout_duration_;
  std::chrono::steady_clock::time_point timeout_start_;
  double ball_moved_threshold_ = 0;

  void SetTimeout(const World & world);

  void SetDistanceThreshold(const World & world);

  bool IsGameStopping(const World & world);

  bool IsStopCommandEnding(const World & world);

  bool IsGameResuming(const World & world);

  bool HasBallMoved(const World & world);

  bool HasTimeoutExpired();
};

}  // namespace ateam_kenobi

#endif  // IN_PLAY_EVAL_HPP_
