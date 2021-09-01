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
#include <ssl_league_msgs/msg/vision_wrapper.hpp>
#include <ssl_league_protobufs/ssl_vision_wrapper.pb.h>

#include "message_conversions.hpp"

namespace ateam_ssl_vision_bridge
{

class SSLVisionBridgeNode : public rclcpp::Node
{
public:
  explicit SSLVisionBridgeNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("ssl_vision_bridge", options),
    multicast_receiver_("224.5.23.2",
      10020,
      [this](auto * buffer, size_t bytes_received) {
        SSL_WrapperPacket vision_proto;
        if (!vision_proto.ParseFromArray(buffer, bytes_received)) {
          vision_publisher_->publish(message_conversions::fromProto(vision_proto));
        } else {
          RCLCPP_INFO(get_logger(), "Failed to parse vision protobuf packet");
        }
      })
  {
    vision_publisher_ = create_publisher<ssl_league_msgs::msg::VisionWrapper>(
      "~/vision_messages",
      rclcpp::SystemDefaultsQoS());
  }

private:
  rclcpp::Publisher<ssl_league_msgs::msg::VisionWrapper>::SharedPtr vision_publisher_;
  ateam_common::MulticastReceiver multicast_receiver_;
};

}  // namespace ateam_ssl_vision_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(ateam_ssl_vision_bridge::SSLVisionBridgeNode)
