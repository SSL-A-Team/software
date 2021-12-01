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

#include <ateam_common/udp_sender.hpp>
#include <ateam_msgs/msg/robot_motion_command.hpp>
#include <ssl_league_protobufs/ssl_simulation_robot_control.pb.h>

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
    udp_sender_("127.0.0.1", 10301)
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

      subscriptions_.at(robot_id) =
        create_subscription<ateam_msgs::msg::RobotMotionCommand>(
        "~/robot_motion_commands/robot" + std::to_string(robot_id),
        10,
        callback);
    }
  }

  void message_callback(
    const ateam_msgs::msg::RobotMotionCommand::SharedPtr robot_commands_msg,
    int robot_id)
  {
    RobotControl robots_control = message_conversions::fromMsg(*robot_commands_msg, robot_id);

    std::string protobuf_msg;
    if (robots_control.SerializeToString(&protobuf_msg)) {
      udp_sender_.send(protobuf_msg.data(), protobuf_msg.size());
    }
  }

private:
  ateam_common::UDPSender udp_sender_;
  std::array<rclcpp::Subscription<ateam_msgs::msg::RobotMotionCommand>::SharedPtr,
    16> subscriptions_;
};

}  // namespace ateam_ssl_simulation_radio_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(ateam_ssl_simulation_radio_bridge::SSLSimulationRadioBridgeNode)
