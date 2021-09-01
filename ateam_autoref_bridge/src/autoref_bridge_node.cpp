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

#include <ateam_common/multicast_receiver.hpp>
#include <ssl_league_msgs/msg/referee.hpp>

#include "message_conversions.hpp"

namespace ateam_autoref_bridge
{

class AutorefBridgeNode : public rclcpp::Node
{
public:
  explicit AutorefBridgeNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("autoref_bridge", options),
    multicast_receiver_("224.5.23.1",
      10003,
      [this](auto * buffer, size_t bytes_received) {
        Referee referee_proto;
        if (referee_proto.ParseFromArray(buffer, bytes_received)) {
          referee_publisher_->publish(message_conversions::fromProto(referee_proto));
        } else {
          RCLCPP_INFO(get_logger(), "Failed to parse referee protobuf packet");
        }
      })
  {
    referee_publisher_ = create_publisher<ssl_league_msgs::msg::Referee>(
      "~/referee_messages",
      rclcpp::SystemDefaultsQoS());
  }

private:
  rclcpp::Publisher<ssl_league_msgs::msg::Referee>::SharedPtr referee_publisher_;
  ateam_common::MulticastReceiver multicast_receiver_;
};

}  // namespace ateam_autoref_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(ateam_autoref_bridge::AutorefBridgeNode)
