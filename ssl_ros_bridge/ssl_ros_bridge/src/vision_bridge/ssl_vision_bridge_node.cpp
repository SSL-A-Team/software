// Copyright 2024 A Team
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

#include <ssl_league_protobufs/ssl_vision_wrapper.pb.h>

#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "core/multicast_receiver.hpp"
#include "core/protobuf_logging.hpp"
#include <ssl_league_msgs/msg/vision_wrapper.hpp>

#include "message_conversions.hpp"

namespace ssl_ros_bridge::vision_bridge
{

class SSLVisionBridgeNode : public rclcpp::Node
{
public:
  explicit SSLVisionBridgeNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("ssl_vision_bridge", options),
    vision_publisher_(create_publisher<ssl_league_msgs::msg::VisionWrapper>("~/vision_messages",
      rclcpp::SystemDefaultsQoS())),
    multicast_receiver_(
      declare_parameter<std::string>("ssl_vision_ip", "224.5.23.2"),
      declare_parameter<int>("ssl_vision_port", 10020),
      std::bind(&SSLVisionBridgeNode::multicastCallback, this, std::placeholders::_3,
      std::placeholders::_4),
      declare_parameter<std::string>("net_interface_address", ""))
  {
    SET_ROS_PROTOBUF_LOG_HANDLER("ssl_vision_bridge.protobuf");
  }

private:
  rclcpp::Publisher<ssl_league_msgs::msg::VisionWrapper>::SharedPtr vision_publisher_;
  core::MulticastReceiver multicast_receiver_;

  void multicastCallback(uint8_t * buffer, size_t bytes_received)
  {
    SSL_WrapperPacket vision_proto;

    // Note: "- 1" is needed due to some weird bug where if the entire buffer
    // is used to do the conversion to protobuf, it would silently fail.
    // But, if all but the last byte is used, it succeeds and at worst some
    // data is lost
    if (!vision_proto.ParseFromArray(buffer, bytes_received - 1)) {
      vision_publisher_->publish(message_conversions::fromProto(vision_proto));
    } else {
      RCLCPP_WARN(get_logger(), "Failed to parse vision protobuf packet");
    }
  }
};

}  // namespace ssl_ros_bridge::vision_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(ssl_ros_bridge::vision_bridge::SSLVisionBridgeNode)
