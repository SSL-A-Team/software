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

#ifndef ATEAM_COMMON__TEAM_COLOR_LISTENER_HPP_
#define ATEAM_COMMON__TEAM_COLOR_LISTENER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <ssl_league_msgs/msg/referee.hpp>
#include <string>

namespace ateam_common
{

/**
 * @brief Utility for subscribing to referee messages and extracting our team color
 *
 */
class TeamColorListener
{
public:
  enum class TeamColor
  {
    Unknown,
    Yellow,
    Blue
  };

  using Callback = std::function<void (TeamColor)>;

  /**
   * @brief Construct a new Team Color Listener object
   *
   * @param node ROS node
   * @param callback Optional callback called on color change
   */
  explicit TeamColorListener(rclcpp::Node & node, Callback callback = {});

  const TeamColor & GetTeamColor() const
  {
    return team_color_;
  }

private:
  const std::string team_name_;
  TeamColor team_color_;
  Callback callback_;
  rclcpp::Subscription<ssl_league_msgs::msg::Referee>::SharedPtr ref_subscription_;

  void RefereeMessageCallback(const ssl_league_msgs::msg::Referee::ConstSharedPtr msg);
};

}  // namespace ateam_common

#endif  // ATEAM_COMMON__TEAM_COLOR_LISTENER_HPP_
