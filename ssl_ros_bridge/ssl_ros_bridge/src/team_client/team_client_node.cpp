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

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "core/protobuf_logging.hpp"
#include <ssl_ros_bridge_msgs/srv/set_desired_keeper.hpp>
#include <ssl_ros_bridge_msgs/srv/substitute_bot.hpp>
#include <ssl_ros_bridge_msgs/srv/reconnect_team_client.hpp>
#include <ssl_ros_bridge_msgs/srv/set_team_advantage_choice.hpp>
#include <ssl_ros_bridge_msgs/msg/team_client_connection_status.hpp>
#include "team_client.hpp"

namespace ssl_ros_bridge::game_controller_bridge
{
class TeamClientNode : public rclcpp::Node
{
public:
  explicit TeamClientNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("team_client_node", options),
    team_client_(get_logger().get_child("team_client"))
  {
    SET_ROS_PROTOBUF_LOG_HANDLER("team_client_node.protobuf");

    declare_parameter<std::string>("gc_ip_address", "");
    declare_parameter<uint16_t>("gc_port", 10008);
    declare_parameter<std::string>("team_name", "A-Team");
    declare_parameter<std::string>("team_color", "auto");

    set_desired_keeper_service_ = create_service<ssl_ros_bridge_msgs::srv::SetDesiredKeeper>(
      "~/set_desired_keeper",
      std::bind(
        &TeamClientNode::HandleSetDesiredKeeper, this, std::placeholders::_1,
        std::placeholders::_2), rclcpp::ServicesQoS());

    substitute_bot_service_ = create_service<ssl_ros_bridge_msgs::srv::SubstituteBot>(
      "~/substitute_bot",
      std::bind(
        &TeamClientNode::HandleSubstituteBot, this, std::placeholders::_1,
        std::placeholders::_2), rclcpp::ServicesQoS());

    reconnect_service_ = create_service<ssl_ros_bridge_msgs::srv::ReconnectTeamClient>(
      "~/reconnect",
      std::bind(
        &TeamClientNode::HandleReconnectTeamClient, this, std::placeholders::_1,
        std::placeholders::_2), rclcpp::ServicesQoS());

    advantage_choice_service_ = create_service<ssl_ros_bridge_msgs::srv::SetTeamAdvantageChoice>(
      "~/set_advantage_choice",
      std::bind(
        &TeamClientNode::HandleSetAdvantageChoice, this, std::placeholders::_1,
        std::placeholders::_2), rclcpp::ServicesQoS());

    connection_status_publisher_ = create_publisher<ssl_ros_bridge_msgs::msg::TeamClientConnectionStatus>(
      "~/connection_status", rclcpp::ServicesQoS());

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
  rclcpp::Service<ssl_ros_bridge_msgs::srv::SetDesiredKeeper>::SharedPtr set_desired_keeper_service_;
  rclcpp::Service<ssl_ros_bridge_msgs::srv::SubstituteBot>::SharedPtr substitute_bot_service_;
  rclcpp::Service<ssl_ros_bridge_msgs::srv::ReconnectTeamClient>::SharedPtr reconnect_service_;
  rclcpp::Service<ssl_ros_bridge_msgs::srv::SetTeamAdvantageChoice>::SharedPtr advantage_choice_service_;
  rclcpp::Publisher<ssl_ros_bridge_msgs::msg::TeamClientConnectionStatus>::SharedPtr
    connection_status_publisher_;
  rclcpp::TimerBase::SharedPtr ping_timer_;

  bool Connect()
  {
    const auto address_string = get_parameter("gc_ip_address").as_string();
    if(address_string.empty()) {
      RCLCPP_WARN(get_logger(), "Address parameter empty. Cannot attempt to connect.");
      // Returning "success" because this isn't a problem during setup.
      return true;
    }
    TeamClient::ConnectionParameters connection_parameters;
    connection_parameters.address = boost::asio::ip::address::from_string(address_string);
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
    const ssl_ros_bridge_msgs::srv::SetDesiredKeeper::Request::SharedPtr request,
    ssl_ros_bridge_msgs::srv::SetDesiredKeeper::Response::SharedPtr response)
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
    const ssl_ros_bridge_msgs::srv::SubstituteBot::Request::SharedPtr /*request*/,
    ssl_ros_bridge_msgs::srv::SubstituteBot::Response::SharedPtr response)
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
    const ssl_ros_bridge_msgs::srv::ReconnectTeamClient::Request::SharedPtr request,
    ssl_ros_bridge_msgs::srv::ReconnectTeamClient::Response::SharedPtr response)
  {
    if(!request->server_address.empty()) {
      set_parameter(rclcpp::Parameter("gc_ip_address", request->server_address));
    }
    response->success = Connect();
  }

  void HandleSetAdvantageChoice(
    const ssl_ros_bridge_msgs::srv::SetTeamAdvantageChoice::Request::SharedPtr request,
    ssl_ros_bridge_msgs::srv::SetTeamAdvantageChoice::Response::SharedPtr response)
  {
    if (!team_client_.IsConnected()) {
      response->success = false;
      response->reason = "Team client is not connected to the Game Controller.";
      RCLCPP_ERROR(
        get_logger(), "Service %s called before team client connected.",
        advantage_choice_service_->get_service_name());
      return;
    }
    TeamClient::AdvantageChoiceOption choice;
    switch (request->choice) {
      case ssl_ros_bridge_msgs::srv::SetTeamAdvantageChoice::Request::STOP:
        choice = TeamClient::AdvantageChoiceOption::Stop;
        break;
      case ssl_ros_bridge_msgs::srv::SetTeamAdvantageChoice::Request::CONTINUE:
        choice = TeamClient::AdvantageChoiceOption::Continue;
        break;
    }
    const auto result = team_client_.SetAdvantageChoice(choice);
    response->success = result.accepted;
    response->reason = result.reason;
  }

  void PingCallback()
  {
    ssl_ros_bridge_msgs::msg::TeamClientConnectionStatus status_msg;
    status_msg.connected = team_client_.IsConnected();
    if (team_client_.IsConnected()) {
      const auto result = team_client_.Ping();
      status_msg.connected = result.request_result.accepted;
      status_msg.ping = rclcpp::Duration(result.ping);
      if(!result.request_result.accepted) {
        team_client_.Disconnect();
        RCLCPP_WARN(get_logger(), "Ping failed. Team client disconnected.");
      }
    }
    connection_status_publisher_->publish(status_msg);
  }
};
}  // namespace ssl_ros_bridge::game_controller_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(ssl_ros_bridge::game_controller_bridge::TeamClientNode)
