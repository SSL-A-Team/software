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
#include <ateam_msgs/srv/set_desired_keeper.hpp>
#include <ateam_msgs/srv/substitute_bot.hpp>

#include "message_conversions.hpp"

namespace ateam_autoref_bridge
{

class AutorefBridgeNode : public rclcpp::Node
{
public:
  explicit AutorefBridgeNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("autoref_bridge", options)
  {
    referee_publisher_ = create_publisher<ssl_league_msgs::msg::Referee>(
      "~/referee_messages",
      rclcpp::SystemDefaultsQoS());

    set_desired_keeper_service_ = create_service<ateam_msgs::srv::SetDesiredKeeper>(
      "~/set_desired_keeper",
      std::bind(
        &AutorefBridgeNode::HandleSetDesiredKeeper, this, std::placeholders::_1,
        std::placeholders::_2), rclcpp::SystemDefaultsQoS().get_rmw_qos_profile());

    substitute_bot_service_ = create_service<ateam_msgs::srv::SubstituteBot>(
      "~/substitute_bot",
      std::bind(
        &AutorefBridgeNode::HandleSubstituteBot, this, std::placeholders::_1,
        std::placeholders::_2), rclcpp::SystemDefaultsQoS().get_rmw_qos_profile());

    const auto multicast_address =
      declare_parameter<std::string>("multicast.address", "224.5.23.1");
    const auto multicast_port = declare_parameter<int>("multicast.port", 10003);
    RCLCPP_INFO(
      get_logger(), "Listening for multicast packets at %s:%ld",
      multicast_address.c_str(), multicast_port);
    multicast_receiver_ = std::make_unique<ateam_common::MulticastReceiver>(
      multicast_address,
      multicast_port, std::bind(
        &AutorefBridgeNode::PublishMulticastMessage, this, std::placeholders::_1,
        std::placeholders::_2));
  }

private:
  rclcpp::Publisher<ssl_league_msgs::msg::Referee>::SharedPtr referee_publisher_;
  rclcpp::Service<ateam_msgs::srv::SetDesiredKeeper>::SharedPtr set_desired_keeper_service_;
  rclcpp::Service<ateam_msgs::srv::SubstituteBot>::SharedPtr substitute_bot_service_;
  std::unique_ptr<ateam_common::MulticastReceiver> multicast_receiver_;

  void PublishMulticastMessage(const char * buffer, const size_t bytes_received)
  {
    Referee referee_proto;
    if (referee_proto.ParseFromArray(buffer, bytes_received)) {
      referee_publisher_->publish(message_conversions::fromProto(referee_proto));
    } else {
      RCLCPP_WARN(get_logger(), "Failed to parse referee protobuf packet");
    }
  }

  void HandleSetDesiredKeeper(
    const ateam_msgs::srv::SetDesiredKeeper::Request::SharedPtr /*request*/,
    ateam_msgs::srv::SetDesiredKeeper::Response::SharedPtr /*response*/)
  {
    RCLCPP_ERROR(get_logger(), "Service %s not implemented yet.", set_desired_keeper_service_->get_service_name());
  }

  void HandleSubstituteBot(
    const ateam_msgs::srv::SubstituteBot::Request::SharedPtr /*request*/,
    ateam_msgs::srv::SubstituteBot::Response::SharedPtr /*response*/)
  {
    RCLCPP_ERROR(get_logger(), "Service %s not implemented yet.", substitute_bot_service_->get_service_name());
  }
};

}  // namespace ateam_autoref_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(ateam_autoref_bridge::AutorefBridgeNode)
