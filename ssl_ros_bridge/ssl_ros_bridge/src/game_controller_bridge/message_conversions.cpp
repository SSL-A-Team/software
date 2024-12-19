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

#include "message_conversions.hpp"
#include <algorithm>
#include <rclcpp/time.hpp>

#define CopyOptional(proto_msg, ros_msg, var_name) \
  if (proto_msg.has_ ## var_name() ) { \
    ros_msg.var_name = {proto_msg.var_name()}; \
  }

#define CopyOptionalStruct(proto_msg, ros_msg, var_name) \
  if (proto_msg.has_ ## var_name() ) { \
    ros_msg.var_name = {fromProto(proto_msg.var_name())}; \
  }

namespace ssl_ros_bridge::game_controller_bridge::message_conversions
{

geometry_msgs::msg::Point32 fromProto(const Vector2 & proto_msg)
{
  geometry_msgs::msg::Point32 ros_msg;
  ros_msg.x = proto_msg.x();
  ros_msg.y = proto_msg.y();
  return ros_msg;
}
geometry_msgs::msg::Point32 fromProto(const Vector3 & proto_msg)
{
  geometry_msgs::msg::Point32 ros_msg;
  ros_msg.x = proto_msg.x();
  ros_msg.y = proto_msg.y();
  ros_msg.z = proto_msg.z();
  return ros_msg;
}

ssl_league_msgs::msg::Referee fromProto(const Referee & proto_msg)
{
  ssl_league_msgs::msg::Referee ros_msg;
  ros_msg.source_identifier = proto_msg.source_identifier();
  ros_msg.match_type = proto_msg.match_type();
  ros_msg.timestamp = rclcpp::Time(proto_msg.packet_timestamp() * 1000);
  ros_msg.stage = proto_msg.stage();
  ros_msg.stage_time_left = proto_msg.stage_time_left();
  ros_msg.command = proto_msg.command();
  ros_msg.command_counter = proto_msg.command_counter();
  ros_msg.command_timestamp = rclcpp::Time(proto_msg.command_timestamp() * 1000);
  ros_msg.yellow = fromProto(proto_msg.yellow());
  ros_msg.blue = fromProto(proto_msg.blue());
  ros_msg.designated_position.x = proto_msg.designated_position().x() / 1000.0;
  ros_msg.designated_position.y = proto_msg.designated_position().y() / 1000.0;
  ros_msg.blue_team_on_positive_half = proto_msg.blue_team_on_positive_half();
  ros_msg.next_command = proto_msg.next_command();
  std::transform(
    proto_msg.game_events().begin(),
    proto_msg.game_events().end(),
    std::back_inserter(ros_msg.game_events),
    [](const auto & p) {return fromProto(p);});
  std::transform(
    proto_msg.game_event_proposals().begin(),
    proto_msg.game_event_proposals().end(),
    std::back_inserter(ros_msg.game_event_proposals),
    [](const auto & p) {return fromProto(p);});
  ros_msg.current_action_time_remaining = proto_msg.current_action_time_remaining();
  ros_msg.status_message = proto_msg.status_message();
  return ros_msg;
}

ssl_league_msgs::msg::TeamInfo fromProto(const Referee::TeamInfo & proto_msg)
{
  ssl_league_msgs::msg::TeamInfo ros_msg;
  ros_msg.name = proto_msg.name();
  ros_msg.score = proto_msg.score();
  ros_msg.red_cards = proto_msg.red_cards();
  std::copy(
    proto_msg.yellow_card_times().begin(),
    proto_msg.yellow_card_times().end(), std::back_inserter(ros_msg.yellow_card_times));
  ros_msg.yellow_cards = proto_msg.yellow_cards();
  ros_msg.timeouts = proto_msg.timeouts();
  ros_msg.timeout_time = proto_msg.timeout_time();
  ros_msg.goalkeeper = proto_msg.goalkeeper();
  ros_msg.foul_counter = proto_msg.foul_counter();
  ros_msg.ball_placement_failures = proto_msg.ball_placement_failures();
  ros_msg.can_place_ball = proto_msg.can_place_ball();
  ros_msg.max_allowed_bots = proto_msg.max_allowed_bots();
  ros_msg.bot_substitution_intent = proto_msg.bot_substitution_intent();
  ros_msg.ball_placement_failures_reached = proto_msg.ball_placement_failures_reached();
  ros_msg.bot_substitution_allowed = proto_msg.bot_substitution_allowed();
  ros_msg.bot_substitutions_left = proto_msg.bot_substitutions_left();
  ros_msg.bot_substitution_time_left = proto_msg.bot_substitution_time_left();
  return ros_msg;
}

ssl_league_msgs::msg::GameEvent fromProto(const GameEvent & proto_msg)
{
  ssl_league_msgs::msg::GameEvent ros_msg;
  ros_msg.id = proto_msg.id();
  ros_msg.type = proto_msg.type();
  std::copy(
    proto_msg.origin().begin(), proto_msg.origin().end(),
    std::back_inserter(ros_msg.origin));
  ros_msg.created_timestamp = proto_msg.created_timestamp();
  CopyOptionalStruct(proto_msg, ros_msg, ball_left_field_touch_line);
  CopyOptionalStruct(proto_msg, ros_msg, ball_left_field_goal_line);
  CopyOptionalStruct(proto_msg, ros_msg, aimless_kick);
  CopyOptionalStruct(proto_msg, ros_msg, attacker_too_close_to_defense_area);
  CopyOptionalStruct(proto_msg, ros_msg, defender_in_defense_area);
  CopyOptionalStruct(proto_msg, ros_msg, boundary_crossing);
  CopyOptionalStruct(proto_msg, ros_msg, keeper_held_ball);
  CopyOptionalStruct(proto_msg, ros_msg, bot_dribbled_ball_too_far);
  CopyOptionalStruct(proto_msg, ros_msg, bot_pushed_bot);
  CopyOptionalStruct(proto_msg, ros_msg, bot_held_ball_deliberately);
  CopyOptionalStruct(proto_msg, ros_msg, bot_tipped_over);
  CopyOptionalStruct(proto_msg, ros_msg, bot_dropped_parts);
  CopyOptionalStruct(proto_msg, ros_msg, attacker_touched_ball_in_defense_area);
  CopyOptionalStruct(proto_msg, ros_msg, bot_kicked_ball_too_fast);
  CopyOptionalStruct(proto_msg, ros_msg, bot_crash_unique);
  CopyOptionalStruct(proto_msg, ros_msg, bot_crash_drawn);
  CopyOptionalStruct(proto_msg, ros_msg, defender_too_close_to_kick_point);
  CopyOptionalStruct(proto_msg, ros_msg, bot_too_fast_in_stop);
  CopyOptionalStruct(proto_msg, ros_msg, bot_interfered_placement);
  CopyOptionalStruct(proto_msg, ros_msg, possible_goal);
  CopyOptionalStruct(proto_msg, ros_msg, goal);
  CopyOptionalStruct(proto_msg, ros_msg, invalid_goal);
  CopyOptionalStruct(proto_msg, ros_msg, attacker_double_touched_ball);
  CopyOptionalStruct(proto_msg, ros_msg, placement_succeeded);
  CopyOptionalStruct(proto_msg, ros_msg, penalty_kick_failed);
  CopyOptionalStruct(proto_msg, ros_msg, no_progress_in_game);
  CopyOptionalStruct(proto_msg, ros_msg, placement_failed);
  CopyOptionalStruct(proto_msg, ros_msg, multiple_cards);
  CopyOptionalStruct(proto_msg, ros_msg, multiple_fouls);
  CopyOptionalStruct(proto_msg, ros_msg, bot_substitution);
  CopyOptionalStruct(proto_msg, ros_msg, excessive_bot_substitution);
  CopyOptionalStruct(proto_msg, ros_msg, too_many_robots);
  CopyOptionalStruct(proto_msg, ros_msg, challenge_flag);
  CopyOptionalStruct(proto_msg, ros_msg, challenge_flag_handled);
  CopyOptionalStruct(proto_msg, ros_msg, emergency_stop);
  CopyOptionalStruct(proto_msg, ros_msg, unsporting_behavior_minor);
  CopyOptionalStruct(proto_msg, ros_msg, unsporting_behavior_major);
  return ros_msg;
}

ssl_league_msgs::msg::GameEventProposalGroup fromProto(const GameEventProposalGroup & proto_msg)
{
  ssl_league_msgs::msg::GameEventProposalGroup ros_msg;
  std::transform(
    proto_msg.game_events().begin(),
    proto_msg.game_events().end(),
    std::back_inserter(ros_msg.game_events),
    [](const auto & p) {return fromProto(p);});
  ros_msg.accepted = proto_msg.accepted();
  return ros_msg;
}

ssl_league_msgs::msg::Division fromProto(const Division & proto_msg)
{
  ssl_league_msgs::msg::Division ros_msg;
  ros_msg.division = proto_msg;
  return ros_msg;
}

ssl_league_msgs::msg::RobotId fromProto(const RobotId & proto_msg)
{
  ssl_league_msgs::msg::RobotId ros_msg;
  CopyOptionalStruct(proto_msg, ros_msg, team);
  CopyOptional(proto_msg, ros_msg, id);
  return ros_msg;
}

ssl_league_msgs::msg::Team fromProto(const Team & proto_msg)
{
  ssl_league_msgs::msg::Team ros_msg;
  ros_msg.color = proto_msg;
  return ros_msg;
}

ssl_league_msgs::msg::AimlessKick fromProto(const GameEvent_AimlessKick & proto_msg)
{
  ssl_league_msgs::msg::AimlessKick ros_msg;
  ros_msg.by_team = fromProto(proto_msg.by_team());
  CopyOptional(proto_msg, ros_msg, by_bot);
  CopyOptionalStruct(proto_msg, ros_msg, location);
  CopyOptionalStruct(proto_msg, ros_msg, kick_location);
  return ros_msg;
}
ssl_league_msgs::msg::AttackerDoubleTouchedBall fromProto(
  const GameEvent_AttackerDoubleTouchedBall & proto_msg)
{
  ssl_league_msgs::msg::AttackerDoubleTouchedBall ros_msg;
  ros_msg.by_team = fromProto(proto_msg.by_team());
  CopyOptional(proto_msg, ros_msg, by_bot);
  CopyOptionalStruct(proto_msg, ros_msg, location);
  return ros_msg;
}
ssl_league_msgs::msg::AttackerTooCloseToDefenseArea fromProto(
  const GameEvent_AttackerTooCloseToDefenseArea & proto_msg)
{
  ssl_league_msgs::msg::AttackerTooCloseToDefenseArea ros_msg;
  ros_msg.by_team = fromProto(proto_msg.by_team());
  CopyOptional(proto_msg, ros_msg, by_bot);
  CopyOptionalStruct(proto_msg, ros_msg, location);
  CopyOptional(proto_msg, ros_msg, distance);
  CopyOptionalStruct(proto_msg, ros_msg, ball_location);
  return ros_msg;
}
ssl_league_msgs::msg::AttackerTouchedBallInDefenseArea fromProto(
  const GameEvent_AttackerTouchedBallInDefenseArea & proto_msg)
{
  ssl_league_msgs::msg::AttackerTouchedBallInDefenseArea ros_msg;
  ros_msg.by_team = fromProto(proto_msg.by_team());
  CopyOptional(proto_msg, ros_msg, by_bot);
  CopyOptionalStruct(proto_msg, ros_msg, location);
  CopyOptional(proto_msg, ros_msg, distance);
  return ros_msg;
}
ssl_league_msgs::msg::AttackerTouchedOpponentInDefenseArea fromProto(
  const GameEvent_AttackerTouchedOpponentInDefenseArea & proto_msg)
{
  ssl_league_msgs::msg::AttackerTouchedOpponentInDefenseArea ros_msg;
  ros_msg.by_team = fromProto(proto_msg.by_team());
  CopyOptional(proto_msg, ros_msg, by_bot);
  CopyOptional(proto_msg, ros_msg, victim);
  CopyOptionalStruct(proto_msg, ros_msg, location);
  return ros_msg;
}
ssl_league_msgs::msg::BallLeftField fromProto(const GameEvent_BallLeftField & proto_msg)
{
  ssl_league_msgs::msg::BallLeftField ros_msg;
  ros_msg.by_team = fromProto(proto_msg.by_team());
  CopyOptional(proto_msg, ros_msg, by_bot);
  CopyOptionalStruct(proto_msg, ros_msg, location);
  return ros_msg;
}
ssl_league_msgs::msg::BotCrashDrawn fromProto(const GameEvent_BotCrashDrawn & proto_msg)
{
  ssl_league_msgs::msg::BotCrashDrawn ros_msg;
  CopyOptional(proto_msg, ros_msg, bot_yellow);
  CopyOptional(proto_msg, ros_msg, bot_blue);
  CopyOptionalStruct(proto_msg, ros_msg, location);
  CopyOptional(proto_msg, ros_msg, crash_speed);
  CopyOptional(proto_msg, ros_msg, speed_diff);
  CopyOptional(proto_msg, ros_msg, crash_angle);
  return ros_msg;
}
ssl_league_msgs::msg::BotCrashUnique fromProto(const GameEvent_BotCrashUnique & proto_msg)
{
  ssl_league_msgs::msg::BotCrashUnique ros_msg;
  ros_msg.by_team = fromProto(proto_msg.by_team());
  CopyOptional(proto_msg, ros_msg, violator);
  CopyOptional(proto_msg, ros_msg, victim);
  CopyOptionalStruct(proto_msg, ros_msg, location);
  CopyOptional(proto_msg, ros_msg, crash_speed);
  CopyOptional(proto_msg, ros_msg, speed_diff);
  CopyOptional(proto_msg, ros_msg, crash_angle);
  return ros_msg;
}
ssl_league_msgs::msg::BotDribbledBallTooFar fromProto(
  const GameEvent_BotDribbledBallTooFar & proto_msg)
{
  ssl_league_msgs::msg::BotDribbledBallTooFar ros_msg;
  ros_msg.by_team = fromProto(proto_msg.by_team());
  CopyOptional(proto_msg, ros_msg, by_bot);
  CopyOptionalStruct(proto_msg, ros_msg, start);
  CopyOptionalStruct(proto_msg, ros_msg, end);
  return ros_msg;
}
ssl_league_msgs::msg::BotHeldBallDeliberately fromProto(
  const GameEvent_BotHeldBallDeliberately & proto_msg)
{
  ssl_league_msgs::msg::BotHeldBallDeliberately ros_msg;
  ros_msg.by_team = fromProto(proto_msg.by_team());
  CopyOptional(proto_msg, ros_msg, by_bot);
  CopyOptionalStruct(proto_msg, ros_msg, location);
  CopyOptional(proto_msg, ros_msg, duration);
  return ros_msg;
}
ssl_league_msgs::msg::BotInterferedPlacement fromProto(
  const GameEvent_BotInterferedPlacement & proto_msg)
{
  ssl_league_msgs::msg::BotInterferedPlacement ros_msg;
  ros_msg.by_team = fromProto(proto_msg.by_team());
  CopyOptional(proto_msg, ros_msg, by_bot);
  CopyOptionalStruct(proto_msg, ros_msg, location);
  return ros_msg;
}
ssl_league_msgs::msg::BotKickedBallTooFast fromProto(
  const GameEvent_BotKickedBallTooFast & proto_msg)
{
  ssl_league_msgs::msg::BotKickedBallTooFast ros_msg;
  ros_msg.by_team = fromProto(proto_msg.by_team());
  CopyOptional(proto_msg, ros_msg, by_bot);
  CopyOptionalStruct(proto_msg, ros_msg, location);
  CopyOptional(proto_msg, ros_msg, initial_ball_speed);
  CopyOptional(proto_msg, ros_msg, chipped);
  return ros_msg;
}
ssl_league_msgs::msg::BotPushedBot fromProto(const GameEvent_BotPushedBot & proto_msg)
{
  ssl_league_msgs::msg::BotPushedBot ros_msg;
  ros_msg.by_team = fromProto(proto_msg.by_team());
  CopyOptional(proto_msg, ros_msg, violator);
  CopyOptional(proto_msg, ros_msg, victim);
  CopyOptionalStruct(proto_msg, ros_msg, location);
  CopyOptional(proto_msg, ros_msg, pushed_distance);
  return ros_msg;
}
ssl_league_msgs::msg::BotSubstitution fromProto(const GameEvent_BotSubstitution & proto_msg)
{
  ssl_league_msgs::msg::BotSubstitution ros_msg;
  ros_msg.by_team = fromProto(proto_msg.by_team());
  return ros_msg;
}
ssl_league_msgs::msg::BotTippedOver fromProto(const GameEvent_BotTippedOver & proto_msg)
{
  ssl_league_msgs::msg::BotTippedOver ros_msg;
  ros_msg.by_team = fromProto(proto_msg.by_team());
  CopyOptional(proto_msg, ros_msg, by_bot);
  CopyOptionalStruct(proto_msg, ros_msg, location);
  CopyOptionalStruct(proto_msg, ros_msg, ball_location);
  return ros_msg;
}
ssl_league_msgs::msg::BotTooFastInStop fromProto(const GameEvent_BotTooFastInStop & proto_msg)
{
  ssl_league_msgs::msg::BotTooFastInStop ros_msg;
  ros_msg.by_team = fromProto(proto_msg.by_team());
  CopyOptional(proto_msg, ros_msg, by_bot);
  CopyOptionalStruct(proto_msg, ros_msg, location);
  CopyOptional(proto_msg, ros_msg, speed);
  return ros_msg;
}
ssl_league_msgs::msg::BoundaryCrossing fromProto(const GameEvent_BoundaryCrossing & proto_msg)
{
  ssl_league_msgs::msg::BoundaryCrossing ros_msg;
  ros_msg.by_team = fromProto(proto_msg.by_team());
  CopyOptionalStruct(proto_msg, ros_msg, location);
  return ros_msg;
}
ssl_league_msgs::msg::ChallengeFlag fromProto(const GameEvent_ChallengeFlag & proto_msg)
{
  ssl_league_msgs::msg::ChallengeFlag ros_msg;
  ros_msg.by_team = fromProto(proto_msg.by_team());
  return ros_msg;
}
ssl_league_msgs::msg::ChippedGoal fromProto(const GameEvent_ChippedGoal & proto_msg)
{
  ssl_league_msgs::msg::ChippedGoal ros_msg;
  ros_msg.by_team = fromProto(proto_msg.by_team());
  CopyOptional(proto_msg, ros_msg, by_bot);
  CopyOptionalStruct(proto_msg, ros_msg, location);
  CopyOptionalStruct(proto_msg, ros_msg, kick_location);
  CopyOptional(proto_msg, ros_msg, max_ball_height);
  return ros_msg;
}
ssl_league_msgs::msg::DefenderInDefenseArea fromProto(
  const GameEvent_DefenderInDefenseArea & proto_msg)
{
  ssl_league_msgs::msg::DefenderInDefenseArea ros_msg;
  ros_msg.by_team = fromProto(proto_msg.by_team());
  CopyOptional(proto_msg, ros_msg, by_bot);
  CopyOptionalStruct(proto_msg, ros_msg, location);
  CopyOptional(proto_msg, ros_msg, distance);
  return ros_msg;
}
ssl_league_msgs::msg::DefenderInDefenseAreaPartially fromProto(
  const GameEvent_DefenderInDefenseAreaPartially & proto_msg)
{
  ssl_league_msgs::msg::DefenderInDefenseAreaPartially ros_msg;
  ros_msg.by_team = fromProto(proto_msg.by_team());
  CopyOptional(proto_msg, ros_msg, by_bot);
  CopyOptionalStruct(proto_msg, ros_msg, location);
  CopyOptional(proto_msg, ros_msg, distance);
  CopyOptionalStruct(proto_msg, ros_msg, ball_location);
  return ros_msg;
}
ssl_league_msgs::msg::DefenderTooCloseToKickPoint fromProto(
  const GameEvent_DefenderTooCloseToKickPoint & proto_msg)
{
  ssl_league_msgs::msg::DefenderTooCloseToKickPoint ros_msg;
  ros_msg.by_team = fromProto(proto_msg.by_team());
  CopyOptional(proto_msg, ros_msg, by_bot);
  CopyOptionalStruct(proto_msg, ros_msg, location);
  CopyOptional(proto_msg, ros_msg, distance);
  return ros_msg;
}
ssl_league_msgs::msg::EmergencyStop fromProto(const GameEvent_EmergencyStop & proto_msg)
{
  ssl_league_msgs::msg::EmergencyStop ros_msg;
  ros_msg.by_team = fromProto(proto_msg.by_team());
  return ros_msg;
}
ssl_league_msgs::msg::Goal fromProto(const GameEvent_Goal & proto_msg)
{
  ssl_league_msgs::msg::Goal ros_msg;
  ros_msg.by_team = fromProto(proto_msg.by_team());
  CopyOptionalStruct(proto_msg, ros_msg, kicking_team);
  CopyOptional(proto_msg, ros_msg, kicking_bot);
  CopyOptionalStruct(proto_msg, ros_msg, location);
  CopyOptionalStruct(proto_msg, ros_msg, kick_location);
  CopyOptional(proto_msg, ros_msg, max_ball_height);
  CopyOptional(proto_msg, ros_msg, num_robots_by_team);
  CopyOptional(proto_msg, ros_msg, last_touch_by_team);
  CopyOptional(proto_msg, ros_msg, message);
  return ros_msg;
}
ssl_league_msgs::msg::IndirectGoal fromProto(const GameEvent_IndirectGoal & proto_msg)
{
  ssl_league_msgs::msg::IndirectGoal ros_msg;
  ros_msg.by_team = fromProto(proto_msg.by_team());
  CopyOptional(proto_msg, ros_msg, by_bot);
  CopyOptionalStruct(proto_msg, ros_msg, location);
  CopyOptionalStruct(proto_msg, ros_msg, kick_location);
  return ros_msg;
}
ssl_league_msgs::msg::KeeperHeldBall fromProto(const GameEvent_KeeperHeldBall & proto_msg)
{
  ssl_league_msgs::msg::KeeperHeldBall ros_msg;
  ros_msg.by_team = fromProto(proto_msg.by_team());
  CopyOptionalStruct(proto_msg, ros_msg, location);
  CopyOptional(proto_msg, ros_msg, duration);
  return ros_msg;
}
ssl_league_msgs::msg::KickTimeout fromProto(const GameEvent_KickTimeout & proto_msg)
{
  ssl_league_msgs::msg::KickTimeout ros_msg;
  ros_msg.by_team = fromProto(proto_msg.by_team());
  CopyOptionalStruct(proto_msg, ros_msg, location);
  CopyOptional(proto_msg, ros_msg, time);
  return ros_msg;
}
ssl_league_msgs::msg::MultipleCards fromProto(const GameEvent_MultipleCards & proto_msg)
{
  ssl_league_msgs::msg::MultipleCards ros_msg;
  ros_msg.by_team = fromProto(proto_msg.by_team());
  return ros_msg;
}
ssl_league_msgs::msg::MultipleFouls fromProto(const GameEvent_MultipleFouls & proto_msg)
{
  ssl_league_msgs::msg::MultipleFouls ros_msg;
  ros_msg.by_team = fromProto(proto_msg.by_team());
  return ros_msg;
}
ssl_league_msgs::msg::MultiplePlacementFailures fromProto(
  const GameEvent_MultiplePlacementFailures & proto_msg)
{
  ssl_league_msgs::msg::MultiplePlacementFailures ros_msg;
  ros_msg.by_team = fromProto(proto_msg.by_team());
  return ros_msg;
}
ssl_league_msgs::msg::NoProgressInGame fromProto(const GameEvent_NoProgressInGame & proto_msg)
{
  ssl_league_msgs::msg::NoProgressInGame ros_msg;
  CopyOptionalStruct(proto_msg, ros_msg, location);
  CopyOptional(proto_msg, ros_msg, time);
  return ros_msg;
}
ssl_league_msgs::msg::PenaltyKickFailed fromProto(const GameEvent_PenaltyKickFailed & proto_msg)
{
  ssl_league_msgs::msg::PenaltyKickFailed ros_msg;
  ros_msg.by_team = fromProto(proto_msg.by_team());
  CopyOptionalStruct(proto_msg, ros_msg, location);
  return ros_msg;
}
ssl_league_msgs::msg::PlacementFailed fromProto(const GameEvent_PlacementFailed & proto_msg)
{
  ssl_league_msgs::msg::PlacementFailed ros_msg;
  ros_msg.by_team = fromProto(proto_msg.by_team());
  CopyOptional(proto_msg, ros_msg, remaining_distance);
  return ros_msg;
}
ssl_league_msgs::msg::PlacementSucceeded fromProto(const GameEvent_PlacementSucceeded & proto_msg)
{
  ssl_league_msgs::msg::PlacementSucceeded ros_msg;
  ros_msg.by_team = fromProto(proto_msg.by_team());
  CopyOptional(proto_msg, ros_msg, time_taken);
  CopyOptional(proto_msg, ros_msg, precision);
  CopyOptional(proto_msg, ros_msg, distance);
  return ros_msg;
}
ssl_league_msgs::msg::Prepared fromProto(const GameEvent_Prepared & proto_msg)
{
  ssl_league_msgs::msg::Prepared ros_msg;
  CopyOptional(proto_msg, ros_msg, time_taken);
  return ros_msg;
}
ssl_league_msgs::msg::TooManyRobots fromProto(const GameEvent_TooManyRobots & proto_msg)
{
  ssl_league_msgs::msg::TooManyRobots ros_msg;
  ros_msg.by_team = fromProto(proto_msg.by_team());
  CopyOptional(proto_msg, ros_msg, num_robots_allowed);
  CopyOptional(proto_msg, ros_msg, num_robots_on_field);
  CopyOptionalStruct(proto_msg, ros_msg, ball_location);
  return ros_msg;
}
ssl_league_msgs::msg::UnsportingBehaviorMajor fromProto(
  const GameEvent_UnsportingBehaviorMajor & proto_msg)
{
  ssl_league_msgs::msg::UnsportingBehaviorMajor ros_msg;
  ros_msg.by_team = fromProto(proto_msg.by_team());
  ros_msg.reason = proto_msg.reason();
  return ros_msg;
}
ssl_league_msgs::msg::UnsportingBehaviorMinor fromProto(
  const GameEvent_UnsportingBehaviorMinor & proto_msg)
{
  ssl_league_msgs::msg::UnsportingBehaviorMinor ros_msg;
  ros_msg.by_team = fromProto(proto_msg.by_team());
  ros_msg.reason = proto_msg.reason();
  return ros_msg;
}
ssl_league_msgs::msg::BotDroppedParts fromProto(const GameEvent_BotDroppedParts & proto_msg)
{
  ssl_league_msgs::msg::BotDroppedParts ros_msg;
  ros_msg.by_team = fromProto(proto_msg.by_team());
  CopyOptional(proto_msg, ros_msg, by_bot);
  CopyOptionalStruct(proto_msg, ros_msg, location);
  CopyOptionalStruct(proto_msg, ros_msg, ball_location);
  return ros_msg;
}
ssl_league_msgs::msg::ChallengeFlagHandled fromProto(
  const GameEvent_ChallengeFlagHandled & proto_msg)
{
  ssl_league_msgs::msg::ChallengeFlagHandled ros_msg;
  ros_msg.by_team = fromProto(proto_msg.by_team());
  ros_msg.accepted = proto_msg.accepted();
  return ros_msg;
}
ssl_league_msgs::msg::ExcessiveBotSubstitution fromProto(
  const GameEvent_ExcessiveBotSubstitution & proto_msg)
{
  ssl_league_msgs::msg::ExcessiveBotSubstitution ros_msg;
  ros_msg.by_team = fromProto(proto_msg.by_team());
  return ros_msg;
}

}  // namespace ssl_ros_bridge::game_controller_bridge::message_conversions
