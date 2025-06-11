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

#ifndef ATEAM_COMMON__GAME_CONTROLLER_LISTENER_HPP_
#define ATEAM_COMMON__GAME_CONTROLLER_LISTENER_HPP_

#include <string>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <ssl_league_msgs/msg/referee.hpp>
#include <ateam_common/topic_names.hpp>
#include <geometry_msgs/msg/point32.hpp>


namespace ateam_common
{

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
  PrepareKickoffOurs = 4,
  PrepareKickoffTheirs = 5,
  PreparePenaltyOurs = 6,
  PreparePenaltyTheirs = 7,
  DirectFreeOurs = 8,
  DirectFreeTheirs = 9,
  IndirectFreeOurs = 10,
  IndirectFreeTheirs = 11,
  TimeoutOurs = 12,
  TimeoutTheirs = 13,
  GoalOurs = 14,
  GoalTheirs = 15,
  BallPlacementOurs = 16,
  BallPlacementTheirs = 17
};

enum class TeamColor
{
  Unknown,
  Yellow,
  Blue
};

enum class TeamSide
{
  Unknown,
  PositiveHalf,
  NegativeHalf
};

/**
 * @brief Utility for subscribing to referee messages and extracting our team color, field side (+, -), current game stage, command, and stage time remaining
 *
 * Users can query the current team color using GameControllerListener::GetTeamColor() or provide a callback to be called
 * when the team color changes.
 *
 * Users can query the current game stage using GameControllerListener::GetGameStage() and the current running
 * command using GameControllerListener::GetGameCommand(). This class does not currently provide a callback to be run
 * on change of either of these items, but could easily be added in the future.
 *
 * This class adds two parameters to the node:
 *
 * - gc_team_name  (string)
 *   The name of our team as it appears in the GC.
 *
 * - default_team_color  (string)
 *   The team color assumed before the first referee message is received. Can be set to 'yellow', 'blue', or 'unknown'
 *
 * - default_team_side  (string) options are positive_half, negative_half
 */
// GameControllerListener
class GameControllerListener
{
public:
  using ColorCallback = std::function<void (TeamColor)>;
  using SideCallback = std::function<void (TeamSide)>;

  /**
   * @brief Construct a new Game State Listener object
   *
   * @param node ROS node
   */
  explicit GameControllerListener(
    rclcpp::Node & node,
    ColorCallback color_callback = {},
    SideCallback side_callback = {}
  );

  const TeamColor & GetTeamColor() const
  {
    return team_color_;
  }

  const TeamSide & GetTeamSide() const
  {
    return team_side_;
  }

  const GameStage & GetGameStage() const
  {
    return game_stage_;
  }

  const GameCommand & GetGameCommand() const
  {
    return game_command_;
  }

  const GameCommand & GetPreviousGameCommand() const
  {
    return prev_game_command_;
  }

  const std::optional<GameCommand> & GetNextGameCommand() const
  {
    return next_game_command_;
  }

  const std::optional<uint32_t> & GetOurGoalieID() const
  {
    return our_goalie_id_;
  }

  const std::optional<uint32_t> & GetTheirGoalieID() const
  {
    return their_goalie_id_;
  }

  const std::optional<geometry_msgs::msg::Point32> & GetDesignatedPosition() const
  {
    return designated_position_;
  }

  const ssl_league_msgs::msg::Referee & GetLatestRefereeMessage() const
  {
    return referee_msg_;
  }

private:
  const std::string team_name_;
  TeamColor team_color_{TeamColor::Unknown};
  TeamSide team_side_{TeamSide::Unknown};
  GameStage game_stage_{GameStage::Unknown};
  GameCommand game_command_{GameCommand::Halt};
  GameCommand prev_game_command_{GameCommand::Halt};
  std::optional<GameCommand> next_game_command_ {};
  std::optional<uint32_t> our_goalie_id_ {};
  std::optional<uint32_t> their_goalie_id_ {};
  std::optional<geometry_msgs::msg::Point32> designated_position_;
  ssl_league_msgs::msg::Referee referee_msg_;

  ColorCallback color_callback_;
  SideCallback side_callback_;

  rclcpp::Subscription<ssl_league_msgs::msg::Referee>::SharedPtr ref_subscription_;

  void RefereeMessageCallback(const ssl_league_msgs::msg::Referee::ConstSharedPtr msg);

  GameCommand ConvertGameCommand(const uint8_t msg_command);
};

}  // namespace ateam_common

#endif  // ATEAM_COMMON__GAME_CONTROLLER_LISTENER_HPP_
