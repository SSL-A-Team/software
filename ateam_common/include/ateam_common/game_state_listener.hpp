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

#ifndef ATEAM_COMMON__GAME_STATE_LISTENER_HPP_
#define ATEAM_COMMON__GAME_STATE_LISTENER_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <ssl_league_msgs/msg/referee.hpp>

namespace ateam_common
{

/**
 * @brief Utility for subscribing to referee messages and extracting the current game stage, command, and stage time remaining
 *
 * The SSL game controller (GC) is the authoritative source for information about the current game state.
 * This utility provides a simple interface for any node to query the current game stage and the current command (play type)
 * that is running. This class subscribes to and parses the referee messages from the GC to check these items.
 *
 * Users can query the current game stage using GameStateListener::GetGameStage() and the current running
 * command using GameStateListener::GetGameCommand(). This class does not currently provide a callback to be run
 * on change of either of these items, but could easily be added in the future.
 */


enum class GameStage
{
  PreFirstHalf = 0,
  FirstHalf = 1,
  Halftime = 2,
  PreSecondHalf = 3,
  SecondHalf = 4,
  ExtraTimeBreak = 5,
  ExtraTimePreFirstHalf = 6,
  ExtraTimeFirstHalf = 7,
  ExtraTimeHalftime = 8,
  ExtraTimePreSecondHalf = 9,
  ExtraTimeSecondHalf = 10,
  PenaltyBreak = 11,
  Penalty = 12,
  PostGame = 13,
  Unknown = 14
};

enum class GameCommand
{
  Halt = 0,
  Stop = 1,
  NormalStart = 2,
  ForceStart = 3,
  PrepareKickoffYellow = 4,
  PrepareKickoffBlue = 5,
  PreparePenaltyYellow = 6,
  PreparePenaltyBlue = 7,
  DirectFreeYellow = 8,
  DirectFreeBlue = 9,
  IndirectFreeYellow = 10,
  IndirectFreeBlue = 11,
  TimeoutYellow = 12,
  TimeoutBlue = 13,
  GoalYellow = 14,
  GoalBlue = 15,
  BallPlacementYellow = 16,
  BallPlacementBlue = 17
};

class GameStateListener
{
public:
  /**
   * @brief Construct a new Game State Listener object
   *
   * @param node ROS node
   */
  explicit GameStateListener(rclcpp::Node & node);

  const GameStage & GetGameStage() const
  {
    return game_stage_;
  }

  const GameCommand & GetGameCommand() const
  {
    return game_command_;
  }

private:
  GameStage game_stage_{GameStage::Unknown};
  GameCommand game_command_{GameCommand::Halt};
  rclcpp::Subscription<ssl_league_msgs::msg::Referee>::SharedPtr ref_subscription_;

  void RefereeMessageCallback(const ssl_league_msgs::msg::Referee::ConstSharedPtr msg);
};

}  // namespace ateam_common

#endif  // ATEAM_COMMON__GAME_STATE_LISTENER_HPP_