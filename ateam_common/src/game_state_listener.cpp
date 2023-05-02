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
  const auto prev_stage_msg = game_stage_;

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
  // Use callback for this instead?
  if (static_cast<GameStage>(msg->stage) != game_stage_){
    game_stage_ = static_cast<GameStage>(msg -> stage);
  }

  // If the stage or command has changed, run the callback
  // if it exists
  if ((game_command_ != prev_command || game_stage_ != prev_stage_msg)&& callback_) {
    callback_(game_stage_, game_command_);
  }
}

}  // namespace ateam_common
