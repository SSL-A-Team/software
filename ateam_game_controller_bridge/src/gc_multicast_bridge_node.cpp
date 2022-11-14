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
#include <ateam_common/protobuf_logging.hpp>
#include <ssl_league_msgs/msg/referee.hpp>
#include <memory>
#include <string>
#include "message_conversions.hpp"

namespace ateam_game_controller_bridge
{

class GCMulticastBridgeNode : public rclcpp::Node
{
public:
  explicit GCMulticastBridgeNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("gc_multicast_bridge", options)
  {
    SET_ROS_PROTOBUF_LOG_HANDLER("gc_multicast_bridge.protobuf");

    referee_publisher_ = create_publisher<ssl_league_msgs::msg::Referee>(
      "~/referee_messages",
      rclcpp::SystemDefaultsQoS());

    const auto multicast_address =
      declare_parameter<std::string>("multicast.address", "224.5.23.1");
    const auto multicast_port = declare_parameter<int>("multicast.port", 10003);
    RCLCPP_INFO(
      get_logger(), "Listening for multicast packets at %s:%ld",
      multicast_address.c_str(), multicast_port);
    multicast_receiver_ = std::make_unique<ateam_common::MulticastReceiver>(
      multicast_address,
      multicast_port, std::bind(
        &GCMulticastBridgeNode::PublishMulticastMessage, this, std::placeholders::_3,
        std::placeholders::_4));
  }

private:
  rclcpp::Publisher<ssl_league_msgs::msg::Referee>::SharedPtr referee_publisher_;
  std::unique_ptr<ateam_common::MulticastReceiver> multicast_receiver_;

  void PublishMulticastMessage(const uint8_t * buffer, const size_t bytes_received)
  {
    Referee referee_proto;
    if (referee_proto.ParseFromArray(buffer, bytes_received)) {
      referee_publisher_->publish(message_conversions::fromProto(referee_proto));
    } else {
      RCLCPP_WARN(get_logger(), "Failed to parse referee protobuf packet");
    }
  }
};

}  // namespace ateam_game_controller_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(ateam_game_controller_bridge::GCMulticastBridgeNode)
