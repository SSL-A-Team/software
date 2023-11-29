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


#ifndef PLAY_SELECTOR_HPP_
#define PLAY_SELECTOR_HPP_

#include "plays/all_plays.hpp"
#include "types/world.hpp"
#include "visualization/overlay_publisher.hpp"
#include "visualization/play_info_publisher.hpp"

namespace ateam_kenobi
{

class PlaySelector
{
public:
  explicit PlaySelector(
    visualization::OverlayPublisher & overlay_publisher,
    visualization::PlayInfoPublisher & play_info_publisher);

  plays::BasePlay * getPlay(const World & world);

private:
  plays::TestPlay test_play_;
  plays::ControlsTestPlay controls_test_play_;
  plays::HaltPlay halt_play_;
  plays::StopPlay stop_play_;
  plays::WallPlay wall_play_;
  plays::OurKickoffPlay our_kickoff_play_;
  plays::TestKickPlay test_kick_play_;
  plays::Basic122 basic_122_play_;
  plays::OurPenaltyPlay our_penalty_play_;
  plays::TheirPenaltyPlay their_penalty_play_;

  ateam_common::GameCommand previous_game_command_ = ateam_common::GameCommand::Halt;
  void * prev_play_address_ = nullptr;
  std::size_t prev_play_type_index_ = -1;

  void resetPlayIfNeeded(plays::BasePlay * play);

  plays::BasePlay * finalizeSelection(
    plays::BasePlay * play,
    ateam_common::GameCommand current_game_command);

  plays::BasePlay * pickNormalStartPlay();
};

}  // namespace ateam_kenobi

#endif  // PLAY_SELECTOR_HPP_
