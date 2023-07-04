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

#include <string>
#include <vector>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ateam_common/topic_names.hpp>
#include <ateam_msgs/msg/robot_motion_command.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace ateam_joystick_control
{

class JoystickControlNode : public rclcpp::Node
{
public:
  explicit JoystickControlNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("joystick_control_node", options)
  {
    joy_subscription_ = create_subscription<sensor_msgs::msg::Joy>(
      std::string(Topics::kJoystick), rclcpp::QoS{1}, std::bind(
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

    declare_parameters<int>(
      "mapping", {
        {"linear.x.axis", 1},
        {"linear.y.axis", 0},
        {"angular.z.axis", 3},
        {"kick", 5},
        {"dribbler.increment", 3},
        {"dribbler.decrement", 2},
        {"dribbler.spin", 4}
      });

    declare_parameters<double>(
      "mapping", {
        {"linear.x.scale", 1.0},
        {"linear.y.scale", 1.0},
        {"angular.z.scale", 1.0},
        {"dribbler.max", 1000.0},
        {"dribbler.min", 0.0}
      });

    // controls the size of steps when changing dribbler speed
    declare_parameter<double>("dribbler_speed_step", 10.0);

    CreatePublisher(declare_parameter<int>("robot_id", 0));
    parameter_callback_handle_ =
      add_on_set_parameters_callback(
      std::bind(
        &JoystickControlNode::SetParameterCallback, this,
        std::placeholders::_1));

    rclcpp::on_shutdown(std::bind_front(&JoystickControlNode::SendStop, this));
  }

private:
  std::string command_topic_template_;
  float dribbler_speed_ = 0.0f;
  sensor_msgs::msg::Joy prev_joy_msg_;
  rclcpp::Publisher<ateam_msgs::msg::RobotMotionCommand>::SharedPtr control_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  void SendStop() {
    if (control_publisher_) {
      ateam_msgs::msg::RobotMotionCommand command_message;
      control_publisher_->publish(command_message);
    }
  }

  void CreatePublisher(const int robot_id)
  {
    if (robot_id == -1) {
      control_publisher_.reset();
    } else{
      std::string topic = command_topic_template_;
      const auto placeholder_pos = topic.find("{}");
      if (placeholder_pos != std::string::npos) {
        topic.replace(placeholder_pos, 2, std::to_string(robot_id));
      }
      control_publisher_ =
        create_publisher<ateam_msgs::msg::RobotMotionCommand>(topic, rclcpp::QoS{1});
    }
  }

  void JoyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_message)
  {

    if (!control_publisher_) {
        return;
    }

    ateam_msgs::msg::RobotMotionCommand command_message;

    command_message.twist.linear.x = get_parameter("mapping.linear.x.scale").as_double() *
      joy_message->axes[get_parameter("mapping.linear.x.axis").as_int()];
    command_message.twist.linear.y = get_parameter("mapping.linear.y.scale").as_double() *
      joy_message->axes[get_parameter("mapping.linear.y.axis").as_int()];
    command_message.twist.angular.z = get_parameter("mapping.angular.z.scale").as_double() *
      joy_message->axes[get_parameter("mapping.angular.z.axis").as_int()];

    const auto kick_button = get_parameter("mapping.kick").as_int();
    command_message.kick = joy_message->buttons[kick_button] && !prev_joy_msg_.buttons[kick_button];

    const auto dribbler_inc_button = get_parameter("mapping.dribbler.increment").as_int();
    const auto dribbler_dec_button = get_parameter("mapping.dribbler.decrement").as_int();

    if (joy_message->buttons[dribbler_inc_button] && !prev_joy_msg_.buttons[dribbler_inc_button]) {
      dribbler_speed_ += get_parameter("dribbler_speed_step").as_double();
    }
    if (joy_message->buttons[dribbler_dec_button] && !prev_joy_msg_.buttons[dribbler_dec_button]) {
      dribbler_speed_ -= get_parameter("dribbler_speed_step").as_double();
    }
    dribbler_speed_ =
      std::clamp(
      dribbler_speed_, static_cast<float>(get_parameter("mapping.dribbler.min").as_double()),
      static_cast<float>(get_parameter("mapping.dribbler.max").as_double()));
    if (joy_message->buttons[get_parameter("mapping.dribbler.spin").as_int()]) {
      command_message.dribbler_speed = dribbler_speed_;
    } else {
      command_message.dribbler_speed = 0.0;
    }

    control_publisher_->publish(command_message);
    prev_joy_msg_ = *joy_message;
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
    if (robot_id < -1 || robot_id > 15) {
      result.successful = false;
      result.reason = "'robot_id' must be in the range [-1,15].";
      return result;
    }

    SendStop();
    CreatePublisher(robot_id);

    result.successful = true;
    return result;
  }
};

}  // namespace ateam_joystick_control

RCLCPP_COMPONENTS_REGISTER_NODE(ateam_joystick_control::JoystickControlNode)
