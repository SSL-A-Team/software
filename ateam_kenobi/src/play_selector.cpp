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
#include <limits>
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
  stp_options.parameter_interface = stp::ParameterInterface(
    "stp_parameters",
    node.get_node_parameters_interface());
  halt_play_ = addPlay<HaltPlay>(stp_options);
  addPlay<TestPlay>(stp_options);
  addPlay<StopPlay>(stp_options);
  addPlay<WallPlay>(stp_options);
  addPlay<KickOnGoalPlay>(stp_options);
  addPlay<OurKickoffPlay>(stp_options);
  addPlay<OurBallPlacementPlay>(stp_options);
  addPlay<TheirBallPlacementPlay>(stp_options);
  addPlay<TestKickPlay>(stp_options);
  addPlay<Basic122>(stp_options);
  addPlay<OurPenaltyPlay>(stp_options);
  addPlay<TestWindowEvalPlay>(stp_options);
  addPlay<TheirKickoffPlay>(stp_options);
  addPlay<TheirPenaltyPlay>(stp_options);
  addPlay<ControlsTestPlay>(stp_options);
  addPlay<DefensePlay>(stp_options);
  addPlay<TrianglePassPlay>(stp_options);
  addPlay<WaypointsPlay>(stp_options);
  addPlay<SpinningAPlay>(stp_options);
  addPlay<PassToLanePlay>(
    "PassLeftForwardPlay", stp_options, play_helpers::lanes::Lane::Left,
    PassToLanePlay::PassDirection::Forward);
  addPlay<PassToLanePlay>(
    "PassCenterForwardPlay", stp_options, play_helpers::lanes::Lane::Center,
    PassToLanePlay::PassDirection::Forward);
  addPlay<PassToLanePlay>(
    "PassRightForwardPlay", stp_options, play_helpers::lanes::Lane::Right,
    PassToLanePlay::PassDirection::Forward);
  addPlay<PassToLanePlay>(
    "PassLeftBackwardPlay", stp_options, play_helpers::lanes::Lane::Left,
    PassToLanePlay::PassDirection::Backward);
  addPlay<PassToLanePlay>(
    "PassCenterBackwardPlay", stp_options, play_helpers::lanes::Lane::Center,
    PassToLanePlay::PassDirection::Backward);
  addPlay<PassToLanePlay>(
    "PassRightBackwardPlay", stp_options, play_helpers::lanes::Lane::Right,
    PassToLanePlay::PassDirection::Backward);
}

stp::Play * PlaySelector::getPlay(const World & world, ateam_msgs::msg::PlaybookState & state_msg)
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

  fillStateMessage(state_msg, world);

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
    plays_, std::back_inserter(play_scores), [this, &world](auto play) {
      void * play_address = static_cast<void *>(play.get());
      // 5% bonus to previous play as hysteresis
      double score_multiplier = (play_address == prev_play_address_) ? 1.05 : 1.0;
      if (!play->isEnabled()) {
        return std::make_pair(play.get(), std::numeric_limits<double>::quiet_NaN());
      }
      return std::make_pair(play.get(), score_multiplier * play->getScore(world));
    });

  auto sort_func = [](const auto & l, const auto & r) {
      // Rank NaN-scored plays low
      if (std::isnan(l.second)) {return true;}
      if (std::isnan(r.second)) {return false;}

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

void PlaySelector::fillStateMessage(ateam_msgs::msg::PlaybookState & msg, const World & world)
{
  msg.override_name = override_play_name_;
  msg.names.reserve(plays_.size());
  msg.enableds.reserve(plays_.size());
  msg.scores.reserve(plays_.size());
  for (const auto & play : plays_) {
    msg.names.push_back(play->getName());
    msg.enableds.push_back(play->isEnabled());
    msg.scores.push_back(play->getScore(world));
  }
}

}  // namespace ateam_kenobi
