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

#include <ssl_league_protobufs/ssl_simulation_robot_control.pb.h>
#include <ssl_league_protobufs/ssl_simulation_robot_feedback.pb.h>

#include <array>
#include <string>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <ateam_common/bi_directional_udp.hpp>
#include <ateam_common/indexed_topic_helpers.hpp>
#include <ateam_common/topic_names.hpp>
#include <ateam_common/game_controller_listener.hpp>
#include <ateam_msgs/msg/robot_feedback.hpp>
#include <ateam_msgs/msg/robot_motion_command.hpp>

#include "message_conversions.hpp"

namespace ateam_ssl_simulation_radio_bridge
{

class SSLSimulationRadioBridgeNode : public rclcpp::Node
{
public:
  explicit SSLSimulationRadioBridgeNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("ateam_ssl_simulation_radio_bridge", options),
    gc_listener_(*this,
      std::bind_front(&SSLSimulationRadioBridgeNode::team_color_change_callback, this))
  {
    declare_parameter("ssl_sim_radio_ip", "127.0.0.1");
    declare_parameter("ssl_sim_blue_port", 10301);
    declare_parameter("ssl_sim_yellow_port", 10302);

    team_color_change_callback(ateam_common::TeamColor::Blue);

    ateam_common::indexed_topic_helpers::create_indexed_subscribers
    <ateam_msgs::msg::RobotMotionCommand>(
      command_subscriptions_,
      Topics::kRobotMotionCommandPrefix,
      rclcpp::SystemDefaultsQoS(),
      &SSLSimulationRadioBridgeNode::message_callback,
      this);

    ateam_common::indexed_topic_helpers::create_indexed_publishers<ateam_msgs::msg::RobotFeedback>(
      feedback_publishers_,
      Topics::kRobotFeedbackPrefix,
      rclcpp::SystemDefaultsQoS(),
      this);
  }

  void team_color_change_callback(const ateam_common::TeamColor color)
  {
    int port = 0;
    switch (color) {
      case ateam_common::TeamColor::Blue:
        port = get_parameter("ssl_sim_blue_port").as_int();
        break;
      case ateam_common::TeamColor::Yellow:
        port = get_parameter("ssl_sim_yellow_port").as_int();
        break;
      case ateam_common::TeamColor::Unknown:
        RCLCPP_WARN(get_logger(), "Unknown team color. Robot radio connection disabled.");
        udp_.reset();
        return;
    }
    RCLCPP_INFO(get_logger(), "Changing radio port to %d", port);
    udp_ = std::make_unique<ateam_common::BiDirectionalUDP>(
      get_parameter(
        "ssl_sim_radio_ip").as_string(), port,
      std::bind_front(&SSLSimulationRadioBridgeNode::feedback_callback, this));
  }

  void message_callback(
    const ateam_msgs::msg::RobotMotionCommand::SharedPtr robot_commands_msg,
    int robot_id)
  {
    if(!udp_) {
      return;
    }
    RobotControl robots_control = message_conversions::fromMsg(*robot_commands_msg, robot_id);

    std::vector<uint8_t> buffer;
    buffer.resize(robots_control.ByteSizeLong());
    if (robots_control.SerializeToArray(buffer.data(), buffer.size())) {
      udp_->send(static_cast<uint8_t *>(buffer.data()), buffer.size());
    }
  }

  void feedback_callback(const uint8_t * buffer, size_t bytes_received)
  {
    if (bytes_received <= 0) {
      // ignore empty feedback packets
      return;
    }
    RobotControlResponse feedback_proto;
    if (!feedback_proto.ParseFromArray(buffer, bytes_received - 1)) {
      for (const auto & single_feedback : feedback_proto.feedback()) {
        int robot_id = single_feedback.id();
        feedback_publishers_.at(robot_id)->publish(message_conversions::fromProto(single_feedback));
      }
    } else {
      RCLCPP_WARN(get_logger(), "Failed to parse robot feedback protobuf packet");
    }
  }

private:
  ateam_common::GameControllerListener gc_listener_;
  std::unique_ptr<ateam_common::BiDirectionalUDP> udp_;
  std::array<rclcpp::Subscription<ateam_msgs::msg::RobotMotionCommand>::SharedPtr,
    16> command_subscriptions_;
  std::array<rclcpp::Publisher<ateam_msgs::msg::RobotFeedback>::SharedPtr, 16> feedback_publishers_;
};

}  // namespace ateam_ssl_simulation_radio_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(ateam_ssl_simulation_radio_bridge::SSLSimulationRadioBridgeNode)
