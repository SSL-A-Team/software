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

#ifndef GAME_CONTROLLER_BRIDGE__MESSAGE_CONVERSIONS_HPP_
#define GAME_CONTROLLER_BRIDGE__MESSAGE_CONVERSIONS_HPP_

#include <ssl_league_protobufs/ssl_gc_referee_message.pb.h>
#include <ssl_league_protobufs/ssl_gc_game_event.pb.h>
#include <ssl_league_protobufs/ssl_gc_common.pb.h>
#include <ssl_league_protobufs/ssl_gc_geometry.pb.h>

#include <ssl_league_msgs/msg/referee.hpp>
#include <ssl_league_msgs/msg/team_info.hpp>
#include <ssl_league_msgs/msg/division.hpp>
#include <ssl_league_msgs/msg/robot_id.hpp>
#include <ssl_league_msgs/msg/team.hpp>
#include <geometry_msgs/msg/point32.hpp>

namespace ssl_ros_bridge::game_controller_bridge::message_conversions
{

geometry_msgs::msg::Point32 fromProto(const Vector2 & proto_msg);
geometry_msgs::msg::Point32 fromProto(const Vector3 & proto_msg);

ssl_league_msgs::msg::Referee fromProto(const Referee & proto_msg);
ssl_league_msgs::msg::TeamInfo fromProto(const Referee::TeamInfo & proto_msg);
ssl_league_msgs::msg::GameEvent fromProto(const GameEvent & proto_msg);
ssl_league_msgs::msg::GameEventProposalGroup fromProto(const GameEventProposalGroup & proto_msg);

ssl_league_msgs::msg::Division fromProto(const Division & proto_msg);
ssl_league_msgs::msg::RobotId fromProto(const RobotId & proto_msg);
ssl_league_msgs::msg::Team fromProto(const Team & proto_msg);

ssl_league_msgs::msg::AimlessKick fromProto(const GameEvent_AimlessKick & proto_msg);
ssl_league_msgs::msg::AttackerDoubleTouchedBall fromProto(
  const GameEvent_AttackerDoubleTouchedBall & proto_msg);
ssl_league_msgs::msg::AttackerTooCloseToDefenseArea fromProto(
  const GameEvent_AttackerTooCloseToDefenseArea & proto_msg);
ssl_league_msgs::msg::AttackerTouchedBallInDefenseArea fromProto(
  const GameEvent_AttackerTouchedBallInDefenseArea & proto_msg);
ssl_league_msgs::msg::AttackerTouchedOpponentInDefenseArea fromProto(
  const GameEvent_AttackerTouchedOpponentInDefenseArea & proto_msg);
ssl_league_msgs::msg::BallLeftField fromProto(const GameEvent_BallLeftField & proto_msg);
ssl_league_msgs::msg::BotCrashDrawn fromProto(const GameEvent_BotCrashDrawn & proto_msg);
ssl_league_msgs::msg::BotCrashUnique fromProto(const GameEvent_BotCrashUnique & proto_msg);
ssl_league_msgs::msg::BotDribbledBallTooFar fromProto(
  const GameEvent_BotDribbledBallTooFar & proto_msg);
ssl_league_msgs::msg::BotHeldBallDeliberately fromProto(
  const GameEvent_BotHeldBallDeliberately & proto_msg);
ssl_league_msgs::msg::BotInterferedPlacement fromProto(
  const GameEvent_BotInterferedPlacement & proto_msg);
ssl_league_msgs::msg::BotKickedBallTooFast fromProto(
  const GameEvent_BotKickedBallTooFast & proto_msg);
ssl_league_msgs::msg::BotPushedBot fromProto(const GameEvent_BotPushedBot & proto_msg);
ssl_league_msgs::msg::BotSubstitution fromProto(const GameEvent_BotSubstitution & proto_msg);
ssl_league_msgs::msg::BotTippedOver fromProto(const GameEvent_BotTippedOver & proto_msg);
ssl_league_msgs::msg::BotTooFastInStop fromProto(const GameEvent_BotTooFastInStop & proto_msg);
ssl_league_msgs::msg::BoundaryCrossing fromProto(const GameEvent_BoundaryCrossing & proto_msg);
ssl_league_msgs::msg::ChallengeFlag fromProto(const GameEvent_ChallengeFlag & proto_msg);
ssl_league_msgs::msg::ChippedGoal fromProto(const GameEvent_ChippedGoal & proto_msg);
ssl_league_msgs::msg::DefenderInDefenseArea fromProto(
  const GameEvent_DefenderInDefenseArea & proto_msg);
ssl_league_msgs::msg::DefenderInDefenseAreaPartially fromProto(
  const GameEvent_DefenderInDefenseAreaPartially & proto_msg);
ssl_league_msgs::msg::DefenderTooCloseToKickPoint fromProto(
  const GameEvent_DefenderTooCloseToKickPoint & proto_msg);
ssl_league_msgs::msg::EmergencyStop fromProto(const GameEvent_EmergencyStop & proto_msg);
ssl_league_msgs::msg::Goal fromProto(const GameEvent_Goal & proto_msg);
ssl_league_msgs::msg::IndirectGoal fromProto(const GameEvent_IndirectGoal & proto_msg);
ssl_league_msgs::msg::KeeperHeldBall fromProto(const GameEvent_KeeperHeldBall & proto_msg);
ssl_league_msgs::msg::KickTimeout fromProto(const GameEvent_KickTimeout & proto_msg);
ssl_league_msgs::msg::MultipleCards fromProto(const GameEvent_MultipleCards & proto_msg);
ssl_league_msgs::msg::MultipleFouls fromProto(const GameEvent_MultipleFouls & proto_msg);
ssl_league_msgs::msg::MultiplePlacementFailures fromProto(
  const GameEvent_MultiplePlacementFailures & proto_msg);
ssl_league_msgs::msg::NoProgressInGame fromProto(const GameEvent_NoProgressInGame & proto_msg);
ssl_league_msgs::msg::PenaltyKickFailed fromProto(const GameEvent_PenaltyKickFailed & proto_msg);
ssl_league_msgs::msg::PlacementFailed fromProto(const GameEvent_PlacementFailed & proto_msg);
ssl_league_msgs::msg::PlacementSucceeded fromProto(const GameEvent_PlacementSucceeded & proto_msg);
ssl_league_msgs::msg::Prepared fromProto(const GameEvent_Prepared & proto_msg);
ssl_league_msgs::msg::TooManyRobots fromProto(const GameEvent_TooManyRobots & proto_msg);
ssl_league_msgs::msg::UnsportingBehaviorMajor fromProto(
  const GameEvent_UnsportingBehaviorMajor & proto_msg);
ssl_league_msgs::msg::UnsportingBehaviorMinor fromProto(
  const GameEvent_UnsportingBehaviorMinor & proto_msg);
ssl_league_msgs::msg::BotDroppedParts fromProto(const GameEvent_BotDroppedParts & proto_msg);
ssl_league_msgs::msg::ChallengeFlagHandled fromProto(
  const GameEvent_ChallengeFlagHandled & proto_msg);
ssl_league_msgs::msg::ExcessiveBotSubstitution fromProto(
  const GameEvent_ExcessiveBotSubstitution & proto_msg);

}  // namespace ssl_ros_bridge::game_controller_bridge::message_conversions

#endif  // GAME_CONTROLLER_BRIDGE__MESSAGE_CONVERSIONS_HPP_
