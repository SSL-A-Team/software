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
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <utility>
#include <vector>
#include <nlohmann/json.hpp>
#include "play_selector.hpp"
#include "plays/all_plays.hpp"
#include "ateam_common/game_controller_listener.hpp"

namespace ateam_kenobi
{

PlaySelector::PlaySelector(rclcpp::Node & node)
: ros_logger_(node.get_logger().get_child("PlaySelector"))
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
  addPlay<KickoffOnGoalPlay>(stp_options);
  addPlay<KickoffPassPlay>(stp_options);
  addPlay<OurKickoffPrepPlay>(stp_options);
  addPlay<OurBallPlacementPlay>(stp_options);
  addPlay<TheirBallPlacementPlay>(stp_options);
  addPlay<TestKickPlay>(stp_options);
  addPlay<Basic122>(stp_options);
  addPlay<OurPenaltyPlay>(stp_options);
  addPlay<TestWindowEvalPlay>(stp_options);
  addPlay<TheirFreeKickPlay>(stp_options);
  addPlay<TheirKickoffPlay>(stp_options);
  addPlay<TheirPenaltyPlay>(stp_options);
  addPlay<ControlsTestPlay>(stp_options);
  addPlay<DefensePlay>(stp_options);
  addPlay<ExtractPlay>(stp_options);
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
  addPlay<TestSpatialMapPlay>(stp_options);
  addPlay<SpatialPassPlay>(stp_options);
}

stp::Play * PlaySelector::getPlay(const World & world, ateam_msgs::msg::PlaybookState & state_msg)
{
  stp::Play * selected_play = nullptr;

  std::vector<double> scores;

  if (world.referee_info.running_command == ateam_common::GameCommand::Halt) {
    selected_play = halt_play_.get();
  }

  if (selected_play == nullptr) {
    selected_play = selectOverridePlay();
  }

  if (selected_play == nullptr) {
    selected_play = selectRankedPlay(world, scores);
  }

  if (selected_play == nullptr) {
    selected_play = halt_play_.get();
  }

  resetPlayIfNeeded(selected_play);

  fillStateMessage(state_msg, scores, selected_play);

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


void PlaySelector::saveToFile(const std::filesystem::path & path)
{
  std::filesystem::create_directories(path.parent_path());
  nlohmann::json data;
  auto plays_arr = nlohmann::json::array();

  std::ranges::transform(plays_, std::back_inserter(plays_arr), [](const auto & play){
      return nlohmann::json{
      {"name", play->getName()},
      {"enabled", play->isEnabled()}
      };
  });

  data["plays"] = plays_arr;
  std::ofstream file(path);
  file << std::setw(2) << data << std::endl;
}

void PlaySelector::loadFromFile(const std::filesystem::path & path)
{
  std::ifstream file(path);
  nlohmann::json data;
  file >> data;

  const auto & json_plays = data["plays"];

  if(!json_plays.is_array()) {
    RCLCPP_ERROR(ros_logger_, "No 'plays' member found in playbook file.");
    return;
  }

  for(const auto & play_settings : json_plays) {
    const auto & name = play_settings["name"];
    if(!name.is_string()) {
      RCLCPP_WARN(ros_logger_, "Skipping play entry with no name.");
      continue;
    }
    auto play = getPlayByName(name);
    if(play == nullptr) {
      RCLCPP_WARN(ros_logger_, "Could not find play with name %s. Skipping.", name.get<std::string>().c_str());
      continue;
    }
    const auto & enabled = play_settings["enabled"];
    if(enabled.is_boolean()) {
      play->setEnabled(enabled);
    }
  }
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

stp::Play * PlaySelector::selectRankedPlay(
  const World & world,
  std::vector<double> & scores_out)
{
  std::vector<std::pair<stp::Play *, double>> play_scores;

  std::ranges::transform(
    plays_, std::back_inserter(play_scores), [this, &world](auto play) {
      void * play_address = static_cast<void *>(play.get());
      double score_multiplier = 1.0;
      if (play_address == prev_play_address_) {
        // 15% bonus to previous play as hysteresis
        score_multiplier = 1.15;
        if (play->getCompletionState() == stp::PlayCompletionState::Busy) {
          // +90% if previous play should not be interrupted
          score_multiplier += 0.9;
        }
      }
      if (!play->isEnabled()) {
        return std::make_pair(play.get(), std::numeric_limits<double>::quiet_NaN());
      }
      return std::make_pair(
        play.get(),
        std::clamp(score_multiplier * play->getScore(world), 0.0, 100.0));
    });

  auto sort_func = [](const auto & l, const auto & r) {
      // Rank NaN-scored plays low
      if (std::isnan(l.second)) {return true;}
      if (std::isnan(r.second)) {return false;}

      return l.second < r.second;
    };

  const auto & max_score = *std::ranges::max_element(play_scores, sort_func);

  std::transform(
    play_scores.begin(), play_scores.end(), std::back_inserter(scores_out), [](const auto & p) {
      return p.second;
    });

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
    if(prev_play_address_ != nullptr) {
      static_cast<stp::Play *>(prev_play_address_)->exit();
    }
    if (play != nullptr) {
      play->reset();
      play->enter();
    }
    prev_play_address_ = play_address;
  } else if (play->getCompletionState() == stp::PlayCompletionState::Done) {
    play->exit();
    play->reset();
    play->enter();
  }
}

void PlaySelector::fillStateMessage(
  ateam_msgs::msg::PlaybookState & msg,
  std::vector<double> & scores,
  const stp::Play * selected_play)
{
  if (scores.empty()) {
    std::fill_n(
      std::back_inserter(scores), plays_.size(),
      std::numeric_limits<double>::quiet_NaN());
  }
  if (scores.size() != plays_.size()) {
    RCLCPP_WARN(
      ros_logger_, "fillStateMessage(): scores.size() != plays_.size(). buffering with NaN");
    std::fill_n(
      std::back_inserter(scores), plays_.size() - scores.size(),
      std::numeric_limits<double>::quiet_NaN());
  }
  msg.override_name = override_play_name_;
  msg.running_play_name = selected_play->getName();

  const auto found_iter = std::find_if(
    plays_.begin(), plays_.end(), [selected_play](const std::shared_ptr<stp::Play> play)-> bool {
      return play.get() == selected_play;
    });
  if (found_iter != plays_.end()) {
    msg.running_play_index = std::distance(plays_.begin(), found_iter);
  } else {
    msg.running_play_index = -1;
  }

  msg.names.reserve(plays_.size());
  msg.enableds.reserve(plays_.size());
  msg.scores.reserve(plays_.size());
  auto ind = 0ul;
  for (const auto & play : plays_) {
    msg.names.push_back(play->getName());
    msg.enableds.push_back(play->isEnabled());
    msg.scores.push_back(scores[ind]);
    ind++;
  }
}

}  // namespace ateam_kenobi
