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

#include "sample_pass_play.hpp"
#include "core/play_helpers/available_robots.hpp"
#include "core/play_helpers/robot_assignment.hpp"
#include "core/play_helpers/shot_evaluation.hpp"

namespace ateam_kenobi::plays
{

// const std::chrono::milliseconds SamplePassPlay::kHoldingTimeout = std::chrono::seconds(3);

SamplePassPlay::SamplePassPlay(stp::Options stp_options)
: stp::Play("SamplePassPlay", stp_options),
  defense_tactic_(createChild<tactics::StandardDefense>("defense")),
  pass_tactic_(createChild<tactics::Pass>("pass")),
  capture_skill_(createChild<skills::Capture>("capture")),
  rand_eng_(std::random_device{}()),
  rho_distribution_(0.0, kSearchRadius),
  theta_distribution_(0.0, 2.0 * M_PI)
{
}

stp::PlayScore SamplePassPlay::getScore(const World & world)
{
  if (!world.in_play &&
    world.referee_info.running_command != ateam_common::GameCommand::ForceStart &&
    world.referee_info.running_command != ateam_common::GameCommand::NormalStart &&
    world.referee_info.running_command != ateam_common::GameCommand::DirectFreeOurs)
  {
    return stp::PlayScore::NaN();
  }

  if(world.referee_info.running_command == ateam_common::GameCommand::DirectFreeOurs &&
    world.ball.pos.x() < world.field.field_length / 4.0)
  {
    return stp::PlayScore::Min();
  }

  auto available_robots = play_helpers::getAvailableRobots(world);
  play_helpers::removeGoalie(available_robots, world);
  if (available_robots.size() < 4) {
    // Not enough robots for this play
    return stp::PlayScore::NaN();
  }

  const ateam_geometry::Rectangle our_defense_area {
    -1 * (world.field.field_length / 2.0),
    -((world.field.defense_area_width / 2.0) + kRobotDiameter),
    -1 * ((world.field.field_length / 2.0) - world.field.defense_area_depth - kRobotDiameter),
    (world.field.defense_area_width / 2.0) + kRobotDiameter
  };
  const ateam_geometry::Rectangle their_defense_area {
    (world.field.field_length / 2.0) - world.field.defense_area_depth - kRobotDiameter,
    -((world.field.defense_area_width / 2.0) + kRobotDiameter),
    world.field.field_length / 2.0,
    (world.field.defense_area_width / 2.0) + kRobotDiameter
  };
  if(ateam_geometry::doIntersect(world.ball.pos,
      our_defense_area) || ateam_geometry::doIntersect(world.ball.pos, their_defense_area))
  {
    return stp::PlayScore::NaN();
  }

  const ateam_geometry::Rectangle field_rect{
    -((world.field.field_length / 2.0) - kRobotDiameter),
    -((world.field.field_width / 2.0) - kRobotDiameter),
    ((world.field.field_length / 2.0) - kRobotDiameter),
    ((world.field.field_width / 2.0) - kRobotDiameter)
  };
  if (!ateam_geometry::doIntersect(field_rect, world.ball.pos)) {
    return stp::PlayScore::NaN();
  }

  return stp::PlayScore(50);
}

stp::PlayCompletionState SamplePassPlay::getCompletionState(const World & world)
{
  if (!pass_locked_) {
    return stp::PlayCompletionState::NotApplicable;
  }
  if (pass_tactic_.isDone()) {
    return stp::PlayCompletionState::Done;
  }
  const ateam_geometry::Rectangle our_defense_area {
    -1 * (world.field.field_length / 2.0),
    -((world.field.defense_area_width / 2.0) + kRobotDiameter),
    -1 * ((world.field.field_length / 2.0) - world.field.defense_area_depth - kRobotDiameter),
    (world.field.defense_area_width / 2.0) + kRobotDiameter
  };
  const ateam_geometry::Rectangle their_defense_area {
    (world.field.field_length / 2.0) - world.field.defense_area_depth - kRobotDiameter,
    -((world.field.defense_area_width / 2.0) + kRobotDiameter),
    world.field.field_length / 2.0,
    (world.field.defense_area_width / 2.0) + kRobotDiameter
  };
  if(ateam_geometry::doIntersect(world.ball.pos,
      our_defense_area) || ateam_geometry::doIntersect(world.ball.pos, their_defense_area))
  {
    return stp::PlayCompletionState::NotApplicable;
  }
  return stp::PlayCompletionState::Busy;
}

void SamplePassPlay::enter()
{
  defense_tactic_.reset();
  pass_tactic_.reset();
  pass_locked_ = false;
  holding_start_time_.reset();
  kicker_id_.reset();
  receiver_id_.reset();
  candidate_receiver_ids_.clear();
  defender_ids_.clear();
  std::fill(target_cache_.begin(), target_cache_.end(), std::nullopt);
  prev_best_score_ = std::numeric_limits<double>::lowest();
  prev_score_change_time_ = std::chrono::steady_clock::now();
}

std::array<std::optional<RobotCommand>, 16> SamplePassPlay::runFrame(const World & world)
{
  std::array<std::optional<RobotCommand>, 16> commands;

  auto available_robots = play_helpers::getAvailableRobots(world);
  play_helpers::removeGoalie(available_robots, world);

  if(kicker_id_ && world.double_touch_forbidden_id_ &&
    *kicker_id_ == *world.double_touch_forbidden_id_)
  {
    kicker_id_.reset();
  }
  if(!kicker_id_) {
    play_helpers::GroupAssignmentSet groups;

    std::vector<int> disallowed_strikers;
    if (world.double_touch_forbidden_id_) {
      disallowed_strikers.push_back(*world.double_touch_forbidden_id_);
    }
    groups.AddPosition("striker", world.ball.pos, disallowed_strikers);
    groups.AddGroup("defense", defense_tactic_.getAssignmentPoints(world));

    const auto assignments = play_helpers::assignGroups(available_robots, groups);

    const auto get_id = [](const auto & r) {return r.id;};

    kicker_id_ = [&assignments]() -> std::optional<int> {
        const auto maybe_kicker = assignments.GetPositionAssignment("striker");
        if (maybe_kicker) {
          return std::make_optional(maybe_kicker.value().id);
        }
        return std::nullopt;
      }();

    defender_ids_.clear();
    std::ranges::transform(assignments.GetGroupFilledAssignments("defense"),
        std::back_inserter(defender_ids_), get_id);

    play_helpers::removeAssignedRobots(available_robots, assignments);

    candidate_receiver_ids_.clear();
    std::ranges::transform(available_robots, std::back_inserter(candidate_receiver_ids_), get_id);
  }

  std::vector<Robot> defenders;
  std::ranges::transform(defender_ids_, std::back_inserter(defenders), [&world](const int id){
      return world.our_robots[id];
                                                                                                                          });
  defense_tactic_.runFrame(world, defenders, commands);
  ForwardPlayInfo(defense_tactic_);

  // Move candidate receivers to best local target
  std::optional<Robot> best_receiver;
  std::optional<ateam_geometry::Point> best_target;
  double best_receiver_score = -1.0;
  for ( const auto & candidate_id : candidate_receiver_ids_ ) {
    const auto candidate = world.our_robots[candidate_id];
    auto [target, score] = getBestPassTargetForCandidate(world, candidate);
    if(target_cache_[candidate_id].has_value()) {
      auto & cache = target_cache_[candidate_id].value();
      if(score > cache.score) {
        cache.score = score;
        cache.target = target;
      } else {
        target = cache.target;
        score = cache.score;
      }
    } else {
      target_cache_[candidate_id] = CacheEntry{target, score};
    }
    RobotCommand command;
    motion::intents::PositionFacing intent;
    intent.position = target;
    intent.face_target = world.ball.pos;
    command.motion_intent = intent;
    commands[candidate.id] = command;
    if (score > best_receiver_score) {
      best_receiver = candidate;
      best_target = target;
      best_receiver_score = score;
    }
  }

  std::optional<double> closest_opponent_distance;
  for ( const auto & opponent : world.their_robots ) {
    if (opponent.visible) {
      const auto sq_dist = CGAL::squared_distance(world.ball.pos, opponent.pos);
      closest_opponent_distance =
        std::min(closest_opponent_distance.value_or(std::numeric_limits<double>::max()), sq_dist);
    }
  }

  if (!pass_locked_) {
    bool should_lock = false;
    if (best_receiver) {
      if (holding_start_time_) {
        if ((std::chrono::steady_clock::now() - *holding_start_time_) > kHoldingTimeout) {
          should_lock = true;
        }
      }

      if(std::abs(best_receiver_score - prev_best_score_) < 1e-2) {
        const auto score_stall_time = std::chrono::steady_clock::now() - prev_score_change_time_;
        if(score_stall_time > kPreemptHoldingScoreStallTime) {
          should_lock = true;
        }
      } else {
        prev_best_score_ = best_receiver_score;
        prev_score_change_time_ = std::chrono::steady_clock::now();
      }

      if (best_receiver_score > kPreemptHoldingScoreThreshold) {
        should_lock = true;
      }

      if(closest_opponent_distance &&
        CGAL::approximate_sqrt(*closest_opponent_distance) <=
        kPreemptHoldingEnemyProximityThreshold)
      {
        should_lock = true;
      }

      if(should_lock) {
        lockPass(*best_receiver, *best_target);
      }
    }

    const auto kicker = world.our_robots[*kicker_id_];

    const auto dist_to_ball = ateam_geometry::norm(kicker.pos - world.ball.pos);
    if (dist_to_ball < (kRobotRadius + kBallRadius + 0.01) ||
      kicker.breakbeam_ball_detected_filtered)
    {
      if (!holding_start_time_) {
        holding_start_time_ = std::chrono::steady_clock::now();
      }
    } else {
      holding_start_time_.reset();
      commands[kicker.id] = capture_skill_.runFrame(world, kicker);
      ForwardPlayInfo(capture_skill_);
    }
  } else {
    const auto kicker = world.our_robots[*kicker_id_];
    const auto receiver = world.our_robots[*receiver_id_];
    RobotCommand kicker_command;
    RobotCommand receiver_command;
    pass_tactic_.runFrame(world, kicker, receiver, kicker_command, receiver_command);
    commands[kicker.id] = kicker_command;
    commands[receiver.id] = receiver_command;
    ForwardPlayInfo(pass_tactic_);
  }

  getPlayInfo()["locked"] = pass_locked_;
  getPlayInfo()["hold time"] = holding_start_time_ ? std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - *holding_start_time_).count() : -1;
  getPlayInfo()["best score"] = best_receiver_score;
  getPlayInfo()["kicker"] = kicker_id_ ? *kicker_id_ : -1;
  getPlayInfo()["receiver"] = receiver_id_ ? *receiver_id_ : -1;
  getPlayInfo()["defenders"] = defender_ids_;
  getPlayInfo()["candidates"] = candidate_receiver_ids_;

  return commands;
}

std::tuple<ateam_geometry::Point,
  double> SamplePassPlay::getBestPassTargetForCandidate(
  const World & world,
  const Robot & candidate)
{
  ateam_geometry::Point best_target;
  double best_score = -1.0;
  for(auto i = 0u; i < kSampleCount; ++i) {
    const auto rho = rho_distribution_(rand_eng_);
    const auto theta = theta_distribution_(rand_eng_);
    const auto dx = rho * std::cos(theta);
    const auto dy = rho * std::sin(theta);
    const auto x = candidate.pos.x() + dx;
    const auto y = candidate.pos.y() + dy;
    const ateam_geometry::Point target{x, y};
    const auto score = getTargetScore(target, world);
    if(std::isnan(score)) {
      continue;
    }
    if (score > best_score) {
      best_target = target;
      best_score = score;
    }
  }
  return {best_target, best_score};
}

double SamplePassPlay::getTargetScore(const ateam_geometry::Point & target, const World & world)
{
  const ateam_geometry::Rectangle field_rect{
    -((world.field.field_length / 2.0) - kRobotDiameter),
    -((world.field.field_width / 2.0) - kRobotDiameter),
    ((world.field.field_length / 2.0) - kRobotDiameter),
    ((world.field.field_width / 2.0) - kRobotDiameter)
  };
  if (!ateam_geometry::doIntersect(field_rect, target)) {
    return std::numeric_limits<double>::lowest();
  }

  const ateam_geometry::Rectangle their_defense_area {
    (world.field.field_length / 2.0) - world.field.defense_area_depth - kRobotDiameter,
    -((world.field.defense_area_width / 2.0) + kRobotDiameter),
    world.field.field_length / 2.0,
    (world.field.defense_area_width / 2.0) + kRobotDiameter
  };
  if (ateam_geometry::doIntersect(their_defense_area, target)) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const ateam_geometry::Rectangle our_defense_area {
    -1 * (world.field.field_length / 2.0),
    -((world.field.defense_area_width / 2.0) + kRobotDiameter),
    -1 * ((world.field.field_length / 2.0) - world.field.defense_area_depth - kRobotDiameter),
    (world.field.defense_area_width / 2.0) + kRobotDiameter
  };
  if (ateam_geometry::doIntersect(our_defense_area, target)) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  double opponent_dist = std::numeric_limits<double>::max();
  const auto pass_segment = ateam_geometry::Segment(world.ball.pos, target);
  for ( const auto & opponent : world.their_robots ) {
    if (opponent.visible) {
      const auto sq_dist = CGAL::squared_distance(pass_segment, opponent.pos);
      opponent_dist = std::min(opponent_dist, sq_dist);
    }
  }

  const auto shot_success_chance = play_helpers::GetShotSuccessChance(world, target);

  std::vector<Robot> friends;
  std::ranges::transform(candidate_receiver_ids_, std::back_inserter(friends),
    [&world](const int id){return world.our_robots[id];});
  const auto closest_friend = play_helpers::getClosestRobot(friends, target);
  const auto friend_dist = ateam_geometry::norm(closest_friend.pos - target);

  const auto friend_proximity_penalty = std::min(kFriendProximityPenalty *
      (friend_dist / kFriendProximityThreshold), 1.0);

  const auto pass_length = ateam_geometry::norm(target - world.ball.pos);
  auto pass_length_factor = 1.0;
  if(pass_length < kMinIdealPassLength) {
    pass_length_factor = std::clamp(1.0 -
        ((kMinIdealPassLength - pass_length) / kMinIdealPassLength), 0.2, 1.0);
  } else if (pass_length > kMaxIdealPassLength) {
    pass_length_factor = std::clamp(1.0 -
        ((pass_length - kMaxIdealPassLength) / kMaxIdealPassLength), 0.2, 1.0);
  }

  return shot_success_chance * opponent_dist * friend_proximity_penalty * pass_length_factor;
}

void SamplePassPlay::lockPass(const Robot & candidate, const ateam_geometry::Point & target)
{
  receiver_id_ = candidate.id;
  pass_tactic_.setTarget(target);
  pass_locked_ = true;
}

}  // namespace ateam_kenobi::plays
