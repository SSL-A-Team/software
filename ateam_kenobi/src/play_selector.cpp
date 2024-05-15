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
#include "plays/all_plays.hpp"
#include "ateam_common/game_controller_listener.hpp"

namespace ateam_kenobi
{

PlaySelector::PlaySelector()
{
  using namespace ateam_kenobi::plays;
  halt_play_ = addPlay<HaltPlay>();
  addPlay<TestPlay>();
  addPlay<StopPlay>();
  addPlay<WallPlay>();
  addPlay<OurKickoffPlay>();
  addPlay<TestKickPlay>();
  addPlay<Basic122>();
  addPlay<OurPenaltyPlay>();
  addPlay<TheirPenaltyPlay>();
  addPlay<ControlsTestPlay>();
  addPlay<TrianglePassPlay>();
  addPlay<WaypointsPlay>();
  addPlay<SpinningAPlay>();
}

plays::BasePlay * PlaySelector::getPlay(const World & world)
{
  plays::BasePlay * selected_play = halt_play_.get();

  std::vector<std::pair<plays::BasePlay *, double>> play_scores;

  std::ranges::transform(
    plays_, std::back_inserter(play_scores), [&world](auto play) {
      return std::make_pair(play.get(), play->getScore(world));
    });

  auto nan_aware_less = [](const auto & l, const auto & r) {
      if (std::isnan(l.second)) {return true;}
      if (std::isnan(r.second)) {return false;}
      return l.second < r.second;
    };

  const auto & max_score = *std::ranges::max_element(play_scores, nan_aware_less);

  if (!std::isnan(max_score.second)) {
    selected_play = max_score.first;
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

}  // namespace ateam_kenobi
