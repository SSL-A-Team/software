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

#include <ssl_league_protobufs/ssl_vision_wrapper.pb.h>

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <ateam_common/multicast_receiver.hpp>
#include <ateam_common/protobuf_logging.hpp>
#include <ateam_common/topic_names.hpp>
#include <ssl_league_msgs/msg/vision_wrapper.hpp>

#include "message_conversions.hpp"

namespace ateam_ssl_vision_bridge
{

class SSLVisionBridgeNode : public rclcpp::Node
{
public:
  explicit SSLVisionBridgeNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("ssl_vision_bridge", options),
    multicast_receiver_("224.5.23.2",
      (declare_parameter("ssl_vision_port", 10020), get_parameter("ssl_vision_port").as_int()),
      [this](const std::string &, const uint16_t, auto * buffer, size_t bytes_received) {
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
    }) {
    SET_ROS_PROTOBUF_LOG_HANDLER("ssl_vision_bridge.protobuf");
    vision_publisher_ = create_publisher<ssl_league_msgs::msg::VisionWrapper>(
      std::string(Topics::kVisionMessages),
      rclcpp::SystemDefaultsQoS());
  }

private:
  rclcpp::Publisher<ssl_league_msgs::msg::VisionWrapper>::SharedPtr vision_publisher_;
  ateam_common::MulticastReceiver multicast_receiver_;
};

}  // namespace ateam_ssl_vision_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(ateam_ssl_vision_bridge::SSLVisionBridgeNode)
