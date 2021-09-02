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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <ateam_common/multicast_sender.hpp>
#include <ssl_league_msgs/msg/vision_wrapper.hpp>
#include <std_msgs/msg/string.hpp>
#include <ssl_league_protobufs/ssl_vision_wrapper.pb.h>

#include "message_conversions.hpp"

namespace ateam_ssl_simulation_radio_bridge
{

class SSLSimulationRadioBridgeNode : public rclcpp::Node
{
public:
  explicit SSLSimulationRadioBridgeNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("ateam_ssl_simulation_radio_bridge", options),
    multicast_sender_("224.5.23.2", 10020)
  {
    auto callback = [&](const std_msgs::msg::String::SharedPtr msg) {
      // Convert to ssl_simulation_robot_control message
      // multicast_sender_.send(msg);
    };

    subscription_ =
      create_subscription<std_msgs::msg::String>(
        "~/robot_motion_commands",
        10,
        callback);
  }

private:
  ateam_common::MulticastSender multicast_sender_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

}  // namespace ateam_ssl_simulation_radio_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(ateam_ssl_simulation_radio_bridge::SSLSimulationRadioBridgeNode)
