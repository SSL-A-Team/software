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

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ateam_common/multicast_receiver.hpp>
#include <ateam_common/protobuf_logging.hpp>
#include <ateam_common/topic_names.hpp>
#include <ateam_msgs/msg/team_client_connection_status.hpp>
#include <ateam_msgs/srv/reconnect_team_client.hpp>
#include <ssl_league_msgs/msg/referee.hpp>
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
      std::string(Topics::kRefereeMessages),
      rclcpp::SystemDefaultsQoS());

    client_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    reconnect_client_ =
      create_client<ateam_msgs::srv::ReconnectTeamClient>("/team_client_node/reconnect",
        rclcpp::ServicesQoS(), client_callback_group_);

    team_client_connection_subscription_ =
      create_subscription<ateam_msgs::msg::TeamClientConnectionStatus>(
        "/team_client_node/connection_status", rclcpp::SystemDefaultsQoS(),
        std::bind(&GCMulticastBridgeNode::TeamClientConnectionStatusCallback, this,
        std::placeholders::_1));

    const auto multicast_address =
      declare_parameter<std::string>("multicast.address", "224.5.23.1");
    const auto multicast_port = declare_parameter<int>("multicast.port", 10003);
    RCLCPP_INFO(
      get_logger(), "Listening for multicast packets at %s:%ld",
      multicast_address.c_str(), multicast_port);
    multicast_receiver_ = std::make_unique<ateam_common::MulticastReceiver>(
      multicast_address,
      multicast_port,
        std::bind(&GCMulticastBridgeNode::PublishMulticastMessage, this, std::placeholders::_1,
        std::placeholders::_3, std::placeholders::_4),
      declare_parameter<std::string>("net_interface_address", ""));
  }

private:
  const std::chrono::seconds kReconnectTimeout{2};
  const std::chrono::seconds kReconnectRetryTime{1};
  bool team_client_connected_ = false;
  std::chrono::steady_clock::time_point last_reconnect_attempt_time_;
  rclcpp::Publisher<ssl_league_msgs::msg::Referee>::SharedPtr referee_publisher_;
  rclcpp::CallbackGroup::SharedPtr client_callback_group_;
  rclcpp::Client<ateam_msgs::srv::ReconnectTeamClient>::SharedPtr reconnect_client_;
  rclcpp::Subscription<ateam_msgs::msg::TeamClientConnectionStatus>::SharedPtr
    team_client_connection_subscription_;
  std::unique_ptr<ateam_common::MulticastReceiver> multicast_receiver_;

  void PublishMulticastMessage(
    const std::string & sender_address, const uint8_t * buffer,
    const size_t bytes_received)
  {
    Referee referee_proto;
    if(!referee_proto.ParseFromArray(buffer, bytes_received)) {
      RCLCPP_WARN(get_logger(), "Failed to parse referee protobuf packet");
      return;
    }
    referee_publisher_->publish(message_conversions::fromProto(referee_proto));
    if(team_client_connected_ || !reconnect_client_->service_is_ready()) {
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    const auto time_since_last_attempt = now - last_reconnect_attempt_time_;
    if(time_since_last_attempt < kReconnectRetryTime) {
      return;
    }
    last_reconnect_attempt_time_ = now;
    auto request = std::make_shared<ateam_msgs::srv::ReconnectTeamClient::Request>();
    request->server_address = sender_address;
    auto service_future = reconnect_client_->async_send_request(request);
    if(service_future.wait_for(kReconnectTimeout) == std::future_status::timeout) {
      RCLCPP_WARN(get_logger(), "Timed out trying to reconnect team client.");
      return;
    }
    if(!service_future.get()->success) {
      RCLCPP_WARN(get_logger(), "Connecting team client to deduced GC server failed.");
      return;
    }
    team_client_connected_ = true;
  }

  void TeamClientConnectionStatusCallback(
    const ateam_msgs::msg::TeamClientConnectionStatus::ConstSharedPtr msg)
  {
    team_client_connected_ = msg->connected;
  }
};

}  // namespace ateam_game_controller_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(ateam_game_controller_bridge::GCMulticastBridgeNode)
