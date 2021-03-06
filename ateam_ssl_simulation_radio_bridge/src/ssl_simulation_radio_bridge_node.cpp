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

#include <ateam_common/bi_directional_udp.hpp>
#include <ateam_msgs/msg/robot_feedback.hpp>
#include <ateam_msgs/msg/robot_motion_command.hpp>
#include <ssl_league_protobufs/ssl_simulation_robot_control.pb.h>
#include <ssl_league_protobufs/ssl_simulation_robot_feedback.pb.h>

#include <array>
#include <string>
#include <functional>

#include "message_conversions.hpp"

namespace ateam_ssl_simulation_radio_bridge
{

class SSLSimulationRadioBridgeNode : public rclcpp::Node
{
public:
  explicit SSLSimulationRadioBridgeNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("ateam_ssl_simulation_radio_bridge", options),
    udp_("127.0.0.1", 10301, std::bind(&SSLSimulationRadioBridgeNode::feedback_callback,
      this,
      std::placeholders::_1,
      std::placeholders::_2))
  {
    for (int robot_id = 0; robot_id < 16; robot_id++) {
      // Full type is required
      // https://answers.ros.org/question/289207/function-callback-using-stdbind-in-ros-2-subscription/
      std::function<void(const ateam_msgs::msg::RobotMotionCommand::SharedPtr)> callback =
        std::bind(
        &SSLSimulationRadioBridgeNode::message_callback,
        this,
        std::placeholders::_1,
        robot_id);

      command_subscriptions_.at(robot_id) =
        create_subscription<ateam_msgs::msg::RobotMotionCommand>(
        "/ateam_ai/robot_motion_commands/robot" + std::to_string(robot_id),
        10,
        callback);

      feedback_publishers_.at(robot_id) = create_publisher<ateam_msgs::msg::RobotFeedback>(
        "~/robot_feedback/robot" + std::to_string(robot_id),
        rclcpp::SystemDefaultsQoS());
    }
  }

  void message_callback(
    const ateam_msgs::msg::RobotMotionCommand::SharedPtr robot_commands_msg,
    int robot_id)
  {
    RobotControl robots_control = message_conversions::fromMsg(*robot_commands_msg, robot_id);

    std::string protobuf_msg;
    if (robots_control.SerializeToString(&protobuf_msg)) {
      udp_.send(protobuf_msg.data(), protobuf_msg.size());
    }
  }

  void feedback_callback(const char * buffer, size_t bytes_received)
  {
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
  ateam_common::BiDirectionalUDP udp_;
  std::array<rclcpp::Subscription<ateam_msgs::msg::RobotMotionCommand>::SharedPtr,
    16> command_subscriptions_;
  std::array<rclcpp::Publisher<ateam_msgs::msg::RobotFeedback>::SharedPtr, 16> feedback_publishers_;
};

}  // namespace ateam_ssl_simulation_radio_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(ateam_ssl_simulation_radio_bridge::SSLSimulationRadioBridgeNode)
