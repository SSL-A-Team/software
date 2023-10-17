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

#include <string>

#include "ateam_common/game_controller_listener.hpp"
#include "ssl_league_msgs/msg/referee.hpp"

namespace ateam_common
{
GameControllerListener::GameControllerListener(
  rclcpp::Node & node,
  ColorCallback color_callback,
  SideCallback side_callback
)
: team_name_(node.declare_parameter<std::string>("gc_team_name", "A-Team")),
  color_callback_(color_callback),
  side_callback_(side_callback)
{
  const auto default_team_color =
    node.declare_parameter<std::string>("default_team_color", "blue");
  if (default_team_color == "yellow") {
    team_color_ = TeamColor::Yellow;
  } else if (default_team_color == "blue") {
    team_color_ = TeamColor::Blue;
  } else if (default_team_color == "unknown") {
    team_color_ = TeamColor::Unknown;
    RCLCPP_WARN(
      node.get_logger(),
      "EXPLICIT UNKNOWN GIVEN FOR TEAM COLOR.");
  } else {
    team_color_ = TeamColor::Unknown;
    RCLCPP_WARN(
      node.get_logger(),
      "Unrecognized value for param 'default_team_color'. Ignoring and defaulting to Unknown.");
  }

  const auto default_team_side =
    node.declare_parameter<std::string>("default_team_side", "negative_half");
  if (default_team_side == "positive_half") {
    team_side_ = TeamSide::PositiveHalf;
  } else if (default_team_side == "negative_half") {
    team_side_ = TeamSide::NegativeHalf;
  } else if (default_team_side == "unknown") {
    team_side_ = TeamSide::Unknown;
  } else {
    RCLCPP_WARN(
      node.get_logger(),
      "Unrecognized value for param 'default_team_side'. Ignoring and defaulting to Unknown.");
  }

  rclcpp::QoS qos(1);
  qos.reliable();
  qos.transient_local();
  ref_subscription_ = node.create_subscription<ssl_league_msgs::msg::Referee>(
    std::string(Topics::kRefereeMessages), qos,
    std::bind(&GameControllerListener::RefereeMessageCallback, this, std::placeholders::_1));
}

void GameControllerListener::RefereeMessageCallback(
  const ssl_league_msgs::msg::Referee::ConstSharedPtr msg)
{
  const auto prev_color = team_color_;
  if (msg->blue.name == team_name_) {
    team_color_ = TeamColor::Blue;
  } else if (msg->yellow.name == team_name_) {
    team_color_ = TeamColor::Yellow;
  } else {
    team_color_ = TeamColor::Unknown;
  }
  if (team_color_ != prev_color && color_callback_) {
    color_callback_(team_color_);
  }

  const auto prev_side = team_side_;
  if (team_color_ == TeamColor::Unknown) {
    team_side_ = TeamSide::Unknown;
  } else {
    bool are_positive_half = !(msg->blue_team_on_positive_half ^ (team_color_ == TeamColor::Blue));
    team_side_ = are_positive_half ? TeamSide::PositiveHalf : TeamSide::NegativeHalf;
    // Our field convention is we should always been on the negative half.
    // So if this is positive for our team we should invert coords
  }
  if (team_side_ != prev_side && side_callback_) {
    side_callback_(team_side_);
  }

  game_stage_ = static_cast<GameStage>(msg->stage);

  uint8_t command = msg->command;

  // Translate the current game command to reflect our team color
  switch (command) {
    case ssl_league_msgs::msg::Referee::COMMAND_HALT:
      game_command_ = GameCommand::Halt;
      break;
    case ssl_league_msgs::msg::Referee::COMMAND_STOP:
      game_command_ = GameCommand::Stop;
      break;
    case ssl_league_msgs::msg::Referee::COMMAND_NORMAL_START:
      game_command_ = GameCommand::NormalStart;
      break;
    case ssl_league_msgs::msg::Referee::COMMAND_FORCE_START:
      game_command_ = GameCommand::ForceStart;
      break;
    case ssl_league_msgs::msg::Referee::COMMAND_PREPARE_KICKOFF_YELLOW:
      game_command_ = team_color_ ==
        TeamColor::Yellow ? GameCommand::PrepareKickoffOurs : GameCommand::PrepareKickoffTheirs;
      break;
    case ssl_league_msgs::msg::Referee::COMMAND_PREPARE_KICKOFF_BLUE:
      game_command_ = team_color_ ==
        TeamColor::Blue ? GameCommand::PrepareKickoffOurs : GameCommand::PrepareKickoffTheirs;
      break;
    case ssl_league_msgs::msg::Referee::COMMAND_PREPARE_PENALTY_YELLOW:
      game_command_ = team_color_ ==
        TeamColor::Yellow ? GameCommand::PreparePenaltyOurs : GameCommand::PreparePenaltyTheirs;
      break;
    case ssl_league_msgs::msg::Referee::COMMAND_PREPARE_PENALTY_BLUE:
      game_command_ = team_color_ ==
        TeamColor::Blue ? GameCommand::PreparePenaltyOurs : GameCommand::PreparePenaltyTheirs;
      break;
    case ssl_league_msgs::msg::Referee::COMMAND_DIRECT_FREE_YELLOW:
      game_command_ = team_color_ ==
        TeamColor::Yellow ? GameCommand::DirectFreeOurs : GameCommand::DirectFreeTheirs;
      break;
    case ssl_league_msgs::msg::Referee::COMMAND_DIRECT_FREE_BLUE:
      game_command_ = team_color_ ==
        TeamColor::Blue ? GameCommand::DirectFreeOurs : GameCommand::DirectFreeTheirs;
      break;
    case ssl_league_msgs::msg::Referee::COMMAND_INDIRECT_FREE_YELLOW:
      game_command_ = team_color_ ==
        TeamColor::Yellow ? GameCommand::IndirectFreeOurs : GameCommand::IndirectFreeTheirs;
      break;
    case ssl_league_msgs::msg::Referee::COMMAND_PREPARE_KICKOFF_BLUE:
      game_command_ = team_color_ ==
        TeamColor::Blue ? GameCommand::IndirectFreeOurs : GameCommand::IndirectFreeTheirs;
      break;
    case ssl_league_msgs::msg::Referee::COMMAND_TIMEOUT_YELLOW:
      game_command_ = team_color_ ==
        TeamColor::Yellow ? GameCommand::TimeoutOurs : GameCommand::TimeoutTheirs;
      break;
    case ssl_league_msgs::msg::Referee::COMMAND_TIMEOUT_BLUE:
      game_command_ = team_color_ ==
        TeamColor::Blue ? GameCommand::TimeoutOurs : GameCommand::TimeoutTheirs;
      break;
    // Note: Command 14 and 15 (goal for yellow/blue) have been deprecated by the league
    case ssl_league_msgs::msg::Referee::COMMAND_GOAL_YELLOW:
      game_command_ = team_color_ ==
        TeamColor::Yellow ? GameCommand::GoalOurs : GameCommand::GoalTheirs;
      break;
    case ssl_league_msgs::msg::Referee::COMMAND_GOAL_BLUE:
      game_command_ = team_color_ ==
        TeamColor::Blue ? GameCommand::GoalOurs : GameCommand::GoalTheirs;
      break;
    case ssl_league_msgs::msg::Referee::COMMAND_BALL_PLACEMENT_YELLOW:
      game_command_ = team_color_ ==
        TeamColor::Yellow ? GameCommand::BallPlacementOurs : GameCommand::BallPlacementTheirs;
      break;
    case ssl_league_msgs::msg::Referee::COMMAND_BALL_PLACEMENT_BLUE:
      game_command_ = team_color_ ==
        TeamColor::Blue ? GameCommand::BallPlacementOurs : GameCommand::BallPlacementTheirs;
      break;
    default:
      game_command_ = GameCommand::Halt;
      break;
  }

  if (team_color_ != TeamColor::Unknown) {
    our_goalie_id_ = team_color_ == TeamColor::Blue ? msg->blue.goalkeeper : msg->yellow.goalkeeper;
    their_goalie_id_ = team_color_ ==
      TeamColor::Blue ? msg->yellow.goalkeeper : msg->blue.goalkeeper;
  } else {
    our_goalie_id_ = std::nullopt;
    their_goalie_id_ = std::nullopt;
  }
}

}  // namespace ateam_common
