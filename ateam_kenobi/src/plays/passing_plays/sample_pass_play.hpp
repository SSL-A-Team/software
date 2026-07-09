// Copyright 2026 A Team
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

#ifndef PLAYS__PASSING_PLAYS__SAMPLE_PASS_PLAY_HPP_
#define PLAYS__PASSING_PLAYS__SAMPLE_PASS_PLAY_HPP_

#include <chrono>
#include <random>
#include <tuple>
#include <ateam_geometry/types.hpp>
#include "core/stp/play.hpp"
#include "tactics/standard_defense.hpp"
#include "tactics/pass.hpp"
#include "skills/capture.hpp"
#include "core/play_helpers/lanes.hpp"

namespace ateam_kenobi::plays
{

class SamplePassPlay : public stp::Play
{
public:
  static constexpr const char * kPlayName = "SamplePassPlay";

  SamplePassPlay(stp::Options stp_options);

  stp::PlayScore getScore(const World & world) override;

  stp::PlayCompletionState getCompletionState(const World &) override;

  void enter() override;

  std::array<std::optional<RobotCommand>, 16> runFrame(const World & world) override;

private:
  static constexpr double kSearchRadius = 1.0;  // m
  static constexpr unsigned int kSampleCount = 20;
  static constexpr std::chrono::milliseconds kHoldingTimeout = std::chrono::seconds(1);
  static constexpr double kPreemptHoldingScoreThreshold = 1000.0;
  static constexpr double kPreemptHoldingEnemyProximityThreshold = kRobotDiameter * 2.0;
  static constexpr std::chrono::milliseconds kPreemptHoldingScoreStallTime =
    std::chrono::milliseconds(150);
  static constexpr double kFriendProximityPenalty = 1.0;
  static constexpr double kFriendProximityThreshold = 3.0;  // m
  static constexpr double kMinIdealPassLength = 1.0;  // m
  static constexpr double kMaxIdealPassLength = 3.5;  // m

  struct CacheEntry
  {
    ateam_geometry::Point target;
    double score;
  };

  tactics::StandardDefense defense_tactic_;
  tactics::Pass pass_tactic_;
  skills::Capture capture_skill_;
  bool pass_locked_ = false;
  std::optional<std::chrono::steady_clock::time_point> holding_start_time_;
  ateam_geometry::Point ball_pos_at_lock_;
  std::default_random_engine rand_eng_;
  std::uniform_real_distribution<> rho_distribution_;
  std::uniform_real_distribution<> theta_distribution_;
  std::optional<int> kicker_id_;
  std::optional<int> receiver_id_;
  std::vector<int> candidate_receiver_ids_;
  std::vector<int> defender_ids_;
  std::array<std::optional<CacheEntry>, 16> target_cache_;
  double prev_best_score_ = std::numeric_limits<double>::lowest();
  std::chrono::steady_clock::time_point prev_score_change_time_;

  std::tuple<ateam_geometry::Point, double> getBestPassTargetForCandidate(
    const World & world,
    const Robot & candidate,
    const play_helpers::lanes::Lane & lane);

  double getTargetScore(
    const ateam_geometry::Point & target, const World & world,
    const play_helpers::lanes::Lane & lane);

  void lockPass(const Robot & candidate, const ateam_geometry::Point & target, const World & world);

};

}  // namespace ateam_kenobi::plays

#endif  // PLAYS__PASSING_PLAYS__SAMPLE_PASS_PLAY_HPP_
