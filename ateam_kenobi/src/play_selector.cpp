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


#include <iostream>
#include "play_selector.hpp"
#include "ateam_common/game_controller_listener.hpp"

namespace ateam_kenobi
{

plays::BasePlay * PlaySelector::getPlay(const World & world)
{
  ateam_common::GameCommand current_game_command = world.referee_info.running_command;

  plays::BasePlay * selected_play = &halt_play_;

  // switch(current_game_command) {
  //   case ateam_common::GameCommand::Halt:
  //   case ateam_common::GameCommand::Stop:
  //     selected_play = &halt_play_;
  //     break;
  //   default:
  //     selected_play = &triangle_pass_play_;
  //     break;
  // }

  switch (current_game_command) {
    case ateam_common::GameCommand::Halt:
    case ateam_common::GameCommand::TimeoutOurs:
    case ateam_common::GameCommand::TimeoutTheirs:
    case ateam_common::GameCommand::GoalOurs:
    case ateam_common::GameCommand::GoalTheirs:
      selected_play = &halt_play_;
      break;
    case ateam_common::GameCommand::Stop:
      selected_play = &stop_play_;
      break;
    case ateam_common::GameCommand::NormalStart:
      selected_play = pickNormalStartPlay(world);
      break;
    case ateam_common::GameCommand::ForceStart:
      selected_play = &basic_122_play_;
      break;
    case ateam_common::GameCommand::PrepareKickoffOurs:
      selected_play = &our_kickoff_play_;
      break;
    case ateam_common::GameCommand::PrepareKickoffTheirs:
      selected_play = &wall_play_;
      break;
    case ateam_common::GameCommand::PreparePenaltyOurs:
      selected_play = &our_penalty_play_;
      break;
    case ateam_common::GameCommand::PreparePenaltyTheirs:
      selected_play = &their_penalty_play_;
      break;
    case ateam_common::GameCommand::DirectFreeOurs:
    case ateam_common::GameCommand::IndirectFreeOurs:
      selected_play = &basic_122_play_;
      break;
    case ateam_common::GameCommand::DirectFreeTheirs:
    case ateam_common::GameCommand::IndirectFreeTheirs:
      if(world.in_play) {
        selected_play = &basic_122_play_;
      } else {
        selected_play = &wall_play_;
      }
      break;
    case ateam_common::GameCommand::BallPlacementOurs:
      selected_play = &ball_placement_play_;
      break;
    case ateam_common::GameCommand::BallPlacementTheirs:
      selected_play = &stop_play_;
      break;
    default:
      std::cerr << "WARNING: Play selector falling through because of unrecognized game command: " << static_cast<int>(current_game_command) << '\n';
      break;
  }

  resetPlayIfNeeded(selected_play);

  return selected_play;
}

void PlaySelector::resetPlayIfNeeded(plays::BasePlay * play)
{
  void * play_address = static_cast<void *>(play);
  if (play_address != prev_play_address_) {
    if (play != nullptr) {
      play->reset();
    }
    prev_play_address_ = play_address;
  }
}

plays::BasePlay * PlaySelector::pickNormalStartPlay(const World & world)
{
  if(world.in_play) {
    return &basic_122_play_;
  }
  switch (world.referee_info.prev_command) {
    case ateam_common::GameCommand::PrepareKickoffOurs:
      return &our_kickoff_play_;
    case ateam_common::GameCommand::PrepareKickoffTheirs:
      return &wall_play_;
    case ateam_common::GameCommand::IndirectFreeOurs:
    case ateam_common::GameCommand::DirectFreeOurs:
      return &basic_122_play_;
      break;
    case ateam_common::GameCommand::IndirectFreeTheirs:
    case ateam_common::GameCommand::DirectFreeTheirs:
      return &wall_play_;
      break;
    case ateam_common::GameCommand::PreparePenaltyOurs:
      return &our_penalty_play_;
      break;
    case ateam_common::GameCommand::PreparePenaltyTheirs:
      return &their_penalty_play_;
      break;
    default:
      return &basic_122_play_;
  }
}

}  // namespace ateam_kenobi
