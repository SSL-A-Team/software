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

#include "ateam_common/game_state_listener.hpp"
#include <string>

namespace ateam_common
{

GameStateListener::GameStateListener(rclcpp::Node & node, Callback callback)
: callback_(callback)
{
  rclcpp::QoS qos(1);
  qos.reliable();
  qos.transient_local();
  ref_subscription_ = node.create_subscription<ssl_league_msgs::msg::Referee>(
    "/gc_multicast_bridge_node/referee_messages", qos,
    std::bind(&GameStateListener::RefereeMessageCallback, this, std::placeholders::_1));
}

void GameStateListener::RefereeMessageCallback(
  const ssl_league_msgs::msg::Referee::ConstSharedPtr msg)
{
  const auto prev_command = game_command_;
  const int prev_stage_msg_ = 0;

  switch (msg->command){
    case 0:
      game_command_ = GameCommand::Halt;
    case 1:
      game_command_ = GameCommand::Stop;
    case 2:
      game_command_ = GameCommand::NormalStart;
    case 3:
      game_command_ = GameCommand::ForceStart;
    case 4:
      game_command_ = GameCommand::PrepareKickoffYellow;
    case 5:
      game_command_ = GameCommand::PrepareKickoffBlue;
    case 6:
      game_command_ = GameCommand::PreparePenaltyYellow;
    case 7:
      game_command_ = GameCommand::PreparePenaltyBlue;
    case 8:
      game_command_ = GameCommand::DirectFreeYellow;
    case 9:
      game_command_ = GameCommand::DirectFreeBlue;
    case 10:
      game_command_ = GameCommand::IndirectFreeYellow;
    case 11:
      game_command_ = GameCommand::IndirectFreeBlue;
    // Simplify to one timeout?
    case 12:
      game_command_ = GameCommand::TimeoutYellow;
    case 13:
      game_command_ = GameCommand::TimeoutBlue;
    case 16:
      game_command_ = GameCommand::BallPlacementYellow;
    case 17:
      game_command_ = GameCommand::BallPlacementBlue;
    // Default to Halt command
    default:
      game_command_ = GameCommand::Halt;
  }

  // Only check the game stage if it has changed
  if (msg->stage != prev_stage_msg_){
    game_stage_ = ConvertNewGameState(msg->stage);
  }

  // If the command has changed, make sure we run the callback
  // if it exists
  if (game_command_ != prev_command && callback_) {
    callback_(game_command_);
  }
}

GameStage GameStateListener::ConvertNewGameState(const int state)
{
  // Convert the int for the game stage to the game stage type
  switch (state){
    case 0:
      return GameStage::PreFirstHalf;
    case 1:
      return GameStage::FirstHalf;
    case 2:
      return GameStage::Halftime;
    case 3:
      return GameStage::PreSecondHalf;
    case 4:
      return GameStage::SecondHalf;
    case 5:
      return GameStage::ExtraTimeBreak;
    case 6:
      return GameStage::ExtraTimePreFirstHalf;
    case 7:
      return GameStage::ExtraTimeFirstHalf;
    case 8:
      return GameStage::ExtraTimeHalftime;
    case 9:
      return GameStage::ExtraTimePreSecondHalf;
    case 10:
      return GameStage::ExtraTimeSecondHalf;
    case 11:
      return GameStage::PenaltyBreak;
    case 12:
      return GameStage::Penalty;
    case 13:
      return GameStage::PostGame;
    default:
      return GameStage::Unknown;
  }
}

}  // namespace ateam_common
