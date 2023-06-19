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

#ifndef ATEAM_COMMON__TEAM_INFO_LISTENER_HPP_
#define ATEAM_COMMON__TEAM_INFO_LISTENER_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <ssl_league_msgs/msg/referee.hpp>

namespace ateam_common
{

/**
 * @brief Utility for subscribing to referee messages and extracting our team color
 *
 * The SSL game controller (GC) is the authoritative source for which color is assigned to which team. This utility
 * provides a simple interface for any node to query our currently assigned team color. This class subscribes to
 * and parses the referee messages from the GC to check our team's color.
 *
 * Users can query the current team color using TeamInfoListener::GetTeamColor() or provide a callback to be called
 * when the team color changes.
 *
 * This class adds two parameters to the node:
 *
 * - gc_team_name  (string)
 *   The name of our team as it appears in the GC.
 *
 * - default_team_color  (string)
 * - default_team_side  (string) options are positive_half, negative_half
 *   The team color assumed before the first referee message is received. Can be set to 'yellow', 'blue', or 'unknown'
 */
class TeamInfoListener
{
public:
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

  using ColorCallback = std::function<void (TeamColor)>;
  using SideCallback = std::function<void (TeamSide)>;

  /**
   * @brief Construct a new Team Color Listener object
   *
   * @param node ROS node
   * @param callback Optional callback called on color change
   */
  explicit TeamInfoListener(
    rclcpp::Node & node, ColorCallback color_callback = {},
    SideCallback side_callback = {});

  const TeamColor & GetTeamColor() const
  {
    return team_color_;
  }

  const TeamSide & GetTeamSide() const
  {
    return team_side_;
  }

private:
  const std::string team_name_;
  TeamColor team_color_{TeamColor::Unknown};
  TeamSide team_side_{TeamSide::Unknown};
  // I feel like we should have just made one callback that returned this object
  ColorCallback color_callback_;
  SideCallback side_callback_;
  rclcpp::Subscription<ssl_league_msgs::msg::Referee>::SharedPtr ref_subscription_;

  void RefereeMessageCallback(const ssl_league_msgs::msg::Referee::ConstSharedPtr msg);
};

}  // namespace ateam_common

#endif  // ATEAM_COMMON__TEAM_INFO_LISTENER_HPP_
