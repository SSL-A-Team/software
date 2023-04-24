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

  if (msg->blue.name == team_name_) {
    team_color_ = TeamColor::Blue;
  } else if (msg->yellow.name == team_name_) {
    team_color_ = TeamColor::Yellow;
  } else {
    team_color_ = TeamColor::Unknown;
  }
  // If the command has changed, make sure we run the callback
  if (game_command_ != prev_command && callback_) {
    callback_(team_color_);
  }
}

}  // namespace ateam_common
