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

#include <algorithm>
#include <iostream>
#include <string>
#include <utility>
#include <vector>
#include "play_selector.hpp"
#include "plays/all_plays.hpp"
#include "ateam_common/game_controller_listener.hpp"

namespace ateam_kenobi
{

PlaySelector::PlaySelector(rclcpp::Node & node)
{
  using namespace ateam_kenobi::plays;  // NOLINT(build/namespaces)
  stp::Options stp_options;
  stp_options.logger = node.get_logger();
  halt_play_ = addPlay<HaltPlay>(stp_options, "HaltPlay");
  addPlay<TestPlay>(stp_options, "TestPlay");
  addPlay<StopPlay>(stp_options, "StopPlay");
  addPlay<WallPlay>(stp_options, "WallPlay");
  addPlay<OurKickoffPlay>(stp_options, "OurKickoffPlay");
  addPlay<TestKickPlay>(stp_options, "TestKickPlay");
  addPlay<Basic122>(stp_options, "Basic122");
  addPlay<OurPenaltyPlay>(stp_options, "OurPenaltyPlay");
  addPlay<TheirPenaltyPlay>(stp_options, "TheirPenaltyPlay");
  addPlay<ControlsTestPlay>(stp_options, "ControlsTestPlay");
  addPlay<TrianglePassPlay>(stp_options, "TrianglePassPlay");
  addPlay<WaypointsPlay>(stp_options, "WaypointsPlay");
  addPlay<SpinningAPlay>(stp_options, "SpinningAPlay");
}

stp::Play * PlaySelector::getPlay(const World & world)
{
  stp::Play * selected_play = nullptr;

  if (world.referee_info.running_command == ateam_common::GameCommand::Halt) {
    selected_play = halt_play_.get();
  }

  if (selected_play == nullptr) {
    selected_play = selectOverridePlay();
  }

  if (selected_play == nullptr) {
    selected_play = selectRankedPlay(world);
  }

  if (selected_play == nullptr) {
    selected_play = halt_play_.get();
  }

  resetPlayIfNeeded(selected_play);

  return selected_play;
}

std::vector<std::string> PlaySelector::getPlayNames()
{
  std::vector<std::string> names;
  std::ranges::transform(
    plays_, std::back_inserter(names), [](const auto p) {
      return p->getName();
    });
  return names;
}

stp::Play * PlaySelector::getPlayByName(const std::string name)
{
  const auto found_iter = std::ranges::find_if(
    plays_, [&name](const auto & play) {
      return play->getName() == name;
    });
  if (found_iter == plays_.end()) {
    return nullptr;
  }
  return found_iter->get();
}

stp::Play * PlaySelector::selectOverridePlay()
{
  if (override_play_name_.empty()) {
    return nullptr;
  }

  const auto found_iter = std::ranges::find_if(
    plays_, [this](const auto & play) {
      return play->getName() == override_play_name_;
    });

  if (found_iter == plays_.end()) {
    return nullptr;
  }

  return found_iter->get();
}

stp::Play * PlaySelector::selectRankedPlay(const World & world)
{
  std::vector<std::pair<stp::Play *, double>> play_scores;

  std::ranges::transform(
    plays_, std::back_inserter(play_scores), [&world](auto play) {
      return std::make_pair(play.get(), play->getScore(world));
    });

  auto sort_func = [](const auto & l, const auto & r) {
      // Rank NaN-scored plays low
      if (std::isnan(l.second)) {return true;}
      if (std::isnan(r.second)) {return false;}

      // Rank disabled plays low
      if (!l.first->isEnabled()) {return true;}
      if (!r.first->isEnabled()) {return false;}

      return l.second < r.second;
    };

  const auto & max_score = *std::ranges::max_element(play_scores, sort_func);

  if (std::isnan(max_score.second)) {
    return nullptr;
  }

  if (!max_score.first->isEnabled()) {
    return nullptr;
  }

  return max_score.first;
}

void PlaySelector::resetPlayIfNeeded(stp::Play * play)
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
