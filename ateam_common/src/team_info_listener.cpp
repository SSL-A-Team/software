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

#include "ateam_common/team_info_listener.hpp"
#include <string>

namespace ateam_common
{

TeamInfoListener::TeamInfoListener(
  rclcpp::Node & node, ColorCallback color_callback,
  SideCallback side_callback)
: team_name_(node.declare_parameter<std::string>("gc_team_name", "A-Team")),
  color_callback_(color_callback),
  side_callback_(side_callback)
{
  const auto default_team_color =
    node.declare_parameter<std::string>("default_team_color", "yellow");
  if (default_team_color == "yellow") {
    team_color_ = TeamColor::Yellow;
  } else if (default_team_color == "blue") {
    team_color_ = TeamColor::Blue;
  } else if (default_team_color == "unknown") {
    team_color_ = TeamColor::Unknown;
  } else {
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
    "/gc_multicast_bridge_node/referee_messages", qos,
    std::bind(&TeamInfoListener::RefereeMessageCallback, this, std::placeholders::_1));
}

void TeamInfoListener::RefereeMessageCallback(
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
    bool are_positive_half = !(msg->blue_team_on_positive_half ^ team_color_ == TeamColor::Blue);
    team_side_ = are_positive_half ? TeamSide::PositiveHalf : TeamSide::NegativeHalf;
    // Our field convention is we should always been on the negative half.
    // So if this is positive for our team we should invert coords
  }
  if (team_side_ != prev_side && side_callback_) {
    side_callback_(team_side_);
  }

}  // namespace ateam_common
