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
#include <ateam_common/protobuf_logging.hpp>
#include <ateam_msgs/srv/set_desired_keeper.hpp>
#include <ateam_msgs/srv/substitute_bot.hpp>
#include <ateam_msgs/srv/reconnect_team_client.hpp>
#include <ateam_msgs/msg/team_client_connection_status.hpp>
#include <string>
#include "team_client.hpp"

namespace ateam_game_controller_bridge
{
class TeamClientNode : public rclcpp::Node
{
public:
  explicit TeamClientNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("team_client_node", options),
    team_client_(get_logger().get_child("team_client"))
  {
    SET_ROS_PROTOBUF_LOG_HANDLER("team_client_node.protobuf");

    if (rcutils_logging_set_logger_level(
        get_logger().get_name(),
        RCUTILS_LOG_SEVERITY_DEBUG) != RCUTILS_RET_OK)
    {
      RCLCPP_ERROR(get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
      rcutils_reset_error();
    }

    declare_parameter<std::string>("gc_ip_address", "127.0.0.1");
    declare_parameter<uint16_t>("gc_port", 10008);
    declare_parameter<std::string>("team_name", "A-Team");
    declare_parameter<std::string>("team_color", "auto");

    set_desired_keeper_service_ = create_service<ateam_msgs::srv::SetDesiredKeeper>(
      "~/set_desired_keeper",
      std::bind(
        &TeamClientNode::HandleSetDesiredKeeper, this, std::placeholders::_1,
        std::placeholders::_2), rclcpp::SystemDefaultsQoS().get_rmw_qos_profile());

    substitute_bot_service_ = create_service<ateam_msgs::srv::SubstituteBot>(
      "~/substitute_bot",
      std::bind(
        &TeamClientNode::HandleSubstituteBot, this, std::placeholders::_1,
        std::placeholders::_2), rclcpp::SystemDefaultsQoS().get_rmw_qos_profile());

    reconnect_service_ = create_service<ateam_msgs::srv::ReconnectTeamClient>(
      "~/reconnect",
      std::bind(
        &TeamClientNode::HandleReconnectTeamClient, this, std::placeholders::_1,
        std::placeholders::_2), rclcpp::SystemDefaultsQoS().get_rmw_qos_profile());

    connection_status_publisher_ = create_publisher<ateam_msgs::msg::TeamClientConnectionStatus>(
      "~/connection_status", rclcpp::SystemDefaultsQoS());

    const auto ping_period = declare_parameter<double>("ping_period", 5);
    ping_timer_ =
      create_wall_timer(
      std::chrono::duration<double>(ping_period),
      std::bind(&TeamClientNode::PingCallback, this));

    if (!Connect()) {
      RCLCPP_ERROR(get_logger(), "Failed to connect to Game Controller.");
    }
  }

private:
  TeamClient team_client_;
  rclcpp::Service<ateam_msgs::srv::SetDesiredKeeper>::SharedPtr set_desired_keeper_service_;
  rclcpp::Service<ateam_msgs::srv::SubstituteBot>::SharedPtr substitute_bot_service_;
  rclcpp::Service<ateam_msgs::srv::ReconnectTeamClient>::SharedPtr reconnect_service_;
  rclcpp::Publisher<ateam_msgs::msg::TeamClientConnectionStatus>::SharedPtr
    connection_status_publisher_;
  rclcpp::TimerBase::SharedPtr ping_timer_;

  bool Connect()
  {
    TeamClient::ConnectionParameters connection_parameters;
    connection_parameters.address =
      boost::asio::ip::address::from_string(get_parameter("gc_ip_address").as_string());
    connection_parameters.port = get_parameter("gc_port").as_int();
    connection_parameters.team_name = get_parameter("team_name").as_string();
    const auto team_color_name = get_parameter("team_color").as_string();
    if (team_color_name == "auto") {
      connection_parameters.team_color = TeamClient::TeamColor::Auto;
    } else if (team_color_name == "blue") {
      connection_parameters.team_color = TeamClient::TeamColor::Blue;
    } else if (team_color_name == "yellow") {
      connection_parameters.team_color = TeamClient::TeamColor::Yellow;
    }
    return team_client_.Connect(connection_parameters);
  }

  void HandleSetDesiredKeeper(
    const ateam_msgs::srv::SetDesiredKeeper::Request::SharedPtr request,
    ateam_msgs::srv::SetDesiredKeeper::Response::SharedPtr response)
  {
    if (!team_client_.IsConnected()) {
      response->success = false;
      response->reason = "Team client is not connected to the Game Controller.";
      RCLCPP_ERROR(
        get_logger(), "Service %s called before team client connected.",
        set_desired_keeper_service_->get_service_name());
      return;
    }
    const auto result = team_client_.SetDesiredKeeper(request->desired_keeper);
    response->success = result.accepted;
    response->reason = result.reason;
  }

  void HandleSubstituteBot(
    const ateam_msgs::srv::SubstituteBot::Request::SharedPtr /*request*/,
    ateam_msgs::srv::SubstituteBot::Response::SharedPtr response)
  {
    if (!team_client_.IsConnected()) {
      response->success = false;
      response->reason = "Team client is not connected to the Game Controller.";
      RCLCPP_ERROR(
        get_logger(), "Service %s called before team client connected.",
        substitute_bot_service_->get_service_name());
      return;
    }
    const auto result = team_client_.RequestBotSubstitution();
    response->success = result.accepted;
    response->reason = result.reason;
  }

  void HandleReconnectTeamClient(
    const ateam_msgs::srv::ReconnectTeamClient::Request::SharedPtr /*request*/,
    ateam_msgs::srv::ReconnectTeamClient::Response::SharedPtr response)
  {
    response->success = Connect();
  }

  void PingCallback()
  {
    ateam_msgs::msg::TeamClientConnectionStatus status_msg;
    status_msg.connected = team_client_.IsConnected();
    if (team_client_.IsConnected()) {
      const auto result = team_client_.Ping();
      status_msg.connected = result.request_result.accepted;
      status_msg.ping = rclcpp::Duration(result.ping);
    }
    connection_status_publisher_->publish(status_msg);
  }
};
}  // namespace ateam_game_controller_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(ateam_game_controller_bridge::TeamClientNode)
