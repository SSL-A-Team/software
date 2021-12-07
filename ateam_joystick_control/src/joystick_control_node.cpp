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
#include <ateam_msgs/msg/robot_motion_command.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <string>
#include <vector>

namespace ateam_joystick_control
{

class JoystickControlNode : public rclcpp::Node
{
public:
  explicit JoystickControlNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("joystick_control_node", options)
  {
    joy_subscription_ = create_subscription<sensor_msgs::msg::Joy>(
      "~/joy", rclcpp::QoS{1}, std::bind(
        &JoystickControlNode::JoyCallback, this,
        std::placeholders::_1));
    command_topic_template_ = declare_parameter<std::string>("command_topic_template", "");
    if (command_topic_template_.empty()) {
      throw std::runtime_error("Missing required parameter 'command_topic_template'");
    }
    if (command_topic_template_.find("{}") == std::string::npos) {
      RCLCPP_WARN(
        get_logger(),
        "Command topic template does not contain '{}' placeholder. Robot ID will have no effect.");
    }
    CreatePublisher(declare_parameter<int>("robot_id", 0));
    parameter_callback_handle_ =
      add_on_set_parameters_callback(
      std::bind(
        &JoystickControlNode::SetParameterCallback, this,
        std::placeholders::_1));
  }

private:
  rclcpp::Publisher<ateam_msgs::msg::RobotMotionCommand>::SharedPtr control_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  std::string command_topic_template_;

  void CreatePublisher(const int robot_id)
  {
    std::string topic = command_topic_template_;
    const auto placeholder_pos = topic.find("{}");
    if (placeholder_pos != std::string::npos) {
      topic.replace(placeholder_pos, 2, std::to_string(robot_id));
    }
    control_publisher_ =
      create_publisher<ateam_msgs::msg::RobotMotionCommand>(topic, rclcpp::QoS{1});
  }

  void JoyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_message)
  {
    ateam_msgs::msg::RobotMotionCommand command_message;

    command_message.twist.linear.x = joy_message->axes[1];
    command_message.twist.linear.y = -1 * joy_message->axes[0];
    command_message.twist.angular.z = joy_message->axes[3];

    // TODO(barulicm) Kick controls
    // TODO(barulicm) Dribbler controls

    control_publisher_->publish(command_message);
  }

  rcl_interfaces::msg::SetParametersResult SetParameterCallback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    const auto robot_id_param_iter = std::find_if(
      parameters.begin(), parameters.end(), [](const auto & param) {
        return param.get_name() == "robot_id";
      });
    if (robot_id_param_iter == parameters.end()) {
      result.successful = true;
      return result;
    }
    const auto robot_id = robot_id_param_iter->as_int();
    if (robot_id < 0 || robot_id > 15) {
      result.successful = false;
      result.reason = "'robot_id' must be in the range [0,15].";
      return result;
    }
    CreatePublisher(robot_id);
    result.successful = true;
    return result;
  }
};

}  // namespace ateam_joystick_control

RCLCPP_COMPONENTS_REGISTER_NODE(ateam_joystick_control::JoystickControlNode)
