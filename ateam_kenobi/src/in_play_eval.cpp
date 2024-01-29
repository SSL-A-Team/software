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

#include "in_play_eval.hpp"

#include <CGAL/squared_distance_2.h>
#include <cmath>
#include <limits>

namespace ateam_kenobi
{

void InPlayEval::Update(World & world)
{
  const auto game_command = world.referee_info.running_command;
  if (game_command == ateam_common::GameCommand::ForceStart) {
    in_play_ = true;
  } else if (IsGameStopped(world)) {
    in_play_ = false;
    ball_start_pos_.reset();
    timeout_duration_.reset();
    timeout_start_ = std::chrono::steady_clock::time_point::max();
  } else if (IsStopCommandEnding(world)) {
    SetTimeout(world);
    SetDistanceThreshold(world);
  } else if (IsGameResuming(world)) {
    ball_start_pos_ = world.ball.pos;
    timeout_start_ = std::chrono::steady_clock::now();
  } else if (!in_play_ && HasTimeoutExpired()) {
    in_play_ = true;
  } else if (!in_play_ && HasBallMoved(world)) {
    in_play_ = true;
  }

  world.in_play = in_play_;
  prev_game_command_ = game_command;
}

void InPlayEval::SetTimeout(const World & world)
{
  switch (world.referee_info.running_command) {
    case ateam_common::GameCommand::PrepareKickoffOurs:
    case ateam_common::GameCommand::PrepareKickoffTheirs:
      timeout_duration_ = std::chrono::seconds(10);
      break;
    case ateam_common::GameCommand::DirectFreeOurs:
    case ateam_common::GameCommand::DirectFreeTheirs:
      // Division A
      // timeout_duration_ = std::chrono::seconds(5);
      // Division B
      timeout_duration_ = std::chrono::seconds(10);
      break;
    default:
      timeout_duration_.reset();
      break;
  }
}

void InPlayEval::SetDistanceThreshold(const World & world)
{
  switch (world.referee_info.running_command) {
    case ateam_common::GameCommand::PrepareKickoffOurs:
    case ateam_common::GameCommand::PrepareKickoffTheirs:
    case ateam_common::GameCommand::DirectFreeOurs:
    case ateam_common::GameCommand::DirectFreeTheirs:
    case ateam_common::GameCommand::PreparePenaltyOurs:
    case ateam_common::GameCommand::PreparePenaltyTheirs:
      ball_moved_threshold_ = 0.05;
      break;
    default:
      ball_moved_threshold_ = std::numeric_limits<double>::infinity();
      break;
  }
}


bool InPlayEval::IsGameStopped(const World & world)
{
  return world.referee_info.running_command == ateam_common::GameCommand::Stop ||
         world.referee_info.running_command == ateam_common::GameCommand::Halt;
}

bool InPlayEval::IsStopCommandEnding(const World & world)
{
  return world.referee_info.running_command != prev_game_command_ &&
         prev_game_command_ == ateam_common::GameCommand::Stop;
}


bool InPlayEval::IsGameResuming(const World & world)
{
  if (world.referee_info.running_command == prev_game_command_) {
    return false;
  }

  if (world.referee_info.running_command == ateam_common::GameCommand::NormalStart ||
    world.referee_info.running_command == ateam_common::GameCommand::DirectFreeOurs ||
    world.referee_info.running_command == ateam_common::GameCommand::DirectFreeTheirs)
  {
    return true;
  }

  return false;
}

bool InPlayEval::HasBallMoved(const World & world)
{
  if (!ball_start_pos_) {
    return false;
  }

  return CGAL::approximate_sqrt(
    CGAL::squared_distance(
      world.ball.pos,
      ball_start_pos_.value())) > ball_moved_threshold_;
}

bool InPlayEval::HasTimeoutExpired()
{
  if (!timeout_duration_) {
    return false;
  }

  return (std::chrono::steady_clock::now() - timeout_duration_.value()) > timeout_start_;
}

}  // namespace ateam_kenobi
