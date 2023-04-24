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
 * This utility provides a simple interface for any node to query the current game stage, the time remaining in that stage,
 * and the current command (play type) that is running. This class subscribes to and parses the referee messages from the GC 
 * to check these items.
 *
 * Users can query the current team color using TeamColorListener::GetTeamColor() or provide a callback to be called
 * when the team color changes.
 *
 * This class adds two parameters to the node:
 *
 * - gc_team_name  (string)
 *   The name of our team as it appears in the GC.
 *
 * - default_team_color  (string)
 *   The team color assumed before the first referee message is received. Can be set to 'yellow', 'blue', or 'unknown'
 */
class GameStateListener
{
public:

  using Callback = std::function<void ()>;

  /**
   * @brief Construct a new Game State Listener object
   *
   * @param node ROS node
   * @param callback Optional callback called on changed game state
   */
  explicit GameStateListener(rclcpp::Node & node, Callback callback = {});

  const GameStage & GetGameStage() const
  {
    return game_stage_;
  }

  const GameCommand & GetGameCommand() const
  {
    return game_command_;
  }

private:
  GameStage game_stage_{GameStage::PreFirstHalf};
  GameCommand game_command_{GameCommand::Halt};
  Callback callback_;
  rclcpp::Subscription<ssl_league_msgs::msg::Referee>::SharedPtr ref_subscription_;

  void RefereeMessageCallback(const ssl_league_msgs::msg::Referee::ConstSharedPtr msg);
};

}  

enum class GameStage
  {
    PreFirstHalf,
    FirstHalf,
    Halftime,
    PreSecondHalf,
    SecondHalf,
    ExtraTimeBreak,
    ExtraTimePreFirstHalf,
    ExtraTimeFirstHalf,
    ExtraTimeHalftime,
    ExtraTimePreSecondHalf,
    ExtraTimeSecondHalf,
    PenaltyBreak,
    Penalty,
    PostGame
  };

enum class GameCommand
  {
    Halt,
    Stop,
    NormalStart,
    ForceStart,
    PrepareKickoffYellow,
    PrepareKickoffBlue,
    PreparePenaltyYellow,
    PreparePenaltyBlue,
    DirectFreeYellow,
    DirectFreeBlue,
    IndirectFreeYellow,
    IndirectFreeBlue,
    TimeoutYellow,
    TimeoutBlue,
    BallPlacementYellow,
    BallPlacementBlue
  };

// namespace ateam_common

#endif  // ATEAM_COMMON__GAME_STATE_LISTENER_HPP_
