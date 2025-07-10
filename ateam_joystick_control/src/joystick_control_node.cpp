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
#include <ateam_msgs/msg/joystick_control_status.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace ateam_joystick_control
{

class JoystickControlNode : public rclcpp::Node
{
public:
  explicit JoystickControlNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("joystick_control_node", options)
  {
    status_publisher_ =
      create_publisher<ateam_msgs::msg::JoystickControlStatus>(std::string(
        Topics::kJoystickControlStatus), rclcpp::QoS(1).transient_local());
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

    linear_x_axis_ = declare_parameter<int>("mapping.linear.x.axis", 1);
    linear_y_axis_ = declare_parameter<int>("mapping.linear.y.axis", 1);
    angular_z_axis_ = declare_parameter<int>("mapping.angular.z.axis", 1);

    kick_trigger_ = ParseTriggerFunction(declare_parameter<std::string>("mapping.kick",
        "axis 5 < -0.8"));
    chip_trigger_ = ParseTriggerFunction(declare_parameter<std::string>("mapping.chip",
        "button 5"));
    dribbler_increment_trigger_ =
      ParseTriggerFunction(declare_parameter<std::string>("mapping.dribbler.increment",
        "button 3"));
    dribbler_decrement_trigger_ =
      ParseTriggerFunction(declare_parameter<std::string>("mapping.dribbler.decrement",
        "button 2"));
    dribbler_spin_trigger_ =
      ParseTriggerFunction(declare_parameter<std::string>("mapping.dribbler.spin", "button 4"));

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

    declare_parameter<double>("kick_speed", 5.0);
    declare_parameter<double>("chip_speed", 5.0);

    CreatePublisher(declare_parameter<int>("robot_id", -1));
    parameter_callback_handle_ =
      add_on_set_parameters_callback(
      std::bind(
        &JoystickControlNode::SetParameterCallback, this,
        std::placeholders::_1));

    rclcpp::on_shutdown(std::bind_front(&JoystickControlNode::SendStop, this));
  }

private:
  using TriggerFunction = std::function<bool(const sensor_msgs::msg::Joy & )>;

  std::string command_topic_template_;
  float dribbler_speed_ = 0.0f;
  sensor_msgs::msg::Joy prev_joy_msg_;

  TriggerFunction kick_trigger_;
  TriggerFunction chip_trigger_;
  TriggerFunction dribbler_increment_trigger_;
  TriggerFunction dribbler_decrement_trigger_;
  TriggerFunction dribbler_spin_trigger_;
  int linear_x_axis_;
  int linear_y_axis_;
  int angular_z_axis_;

  rclcpp::Publisher<ateam_msgs::msg::RobotMotionCommand>::SharedPtr control_publisher_;
  rclcpp::Publisher<ateam_msgs::msg::JoystickControlStatus>::SharedPtr status_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  void SendStop()
  {
    if (control_publisher_) {
      ateam_msgs::msg::RobotMotionCommand command_message;
      control_publisher_->publish(command_message);
    }
  }

  void CreatePublisher(const int robot_id)
  {
    ateam_msgs::msg::JoystickControlStatus status_msg;
    if (robot_id == -1) {
      control_publisher_.reset();
      status_msg.is_active = false;
      status_msg.active_id = -1;
    } else {
      std::string topic = command_topic_template_;
      const auto placeholder_pos = topic.find("{}");
      if (placeholder_pos != std::string::npos) {
        topic.replace(placeholder_pos, 2, std::to_string(robot_id));
      }
      control_publisher_ =
        create_publisher<ateam_msgs::msg::RobotMotionCommand>(topic,
          rclcpp::SystemDefaultsQoS().keep_last(1));
      status_msg.is_active = true;
      status_msg.active_id = robot_id;
    }
    status_publisher_->publish(status_msg);
  }

  void JoyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_message)
  {
    if (!control_publisher_) {
      return;
    }

    ateam_msgs::msg::RobotMotionCommand command_message;

    command_message.twist.linear.x = get_parameter("mapping.linear.x.scale").as_double() *
      joy_message->axes[linear_x_axis_];
    command_message.twist.linear.y = get_parameter("mapping.linear.y.scale").as_double() *
      joy_message->axes[linear_y_axis_];
    command_message.twist.angular.z = get_parameter("mapping.angular.z.scale").as_double() *
      joy_message->axes[angular_z_axis_];

    if (kick_trigger_(*joy_message)) {
      command_message.kick_request = ateam_msgs::msg::RobotMotionCommand::KR_KICK_TOUCH;
      command_message.kick_speed = get_parameter("kick_speed").as_double();
    } else if (chip_trigger_(*joy_message)) {
      command_message.kick_request = ateam_msgs::msg::RobotMotionCommand::KR_CHIP_TOUCH;
      command_message.kick_speed = get_parameter("chip_speed").as_double();
    } else {
      command_message.kick_request = ateam_msgs::msg::RobotMotionCommand::KR_ARM;
    }

    if (dribbler_increment_trigger_(*joy_message) && !dribbler_increment_trigger_(prev_joy_msg_)) {
      dribbler_speed_ += get_parameter("dribbler_speed_step").as_double();
    }
    if (dribbler_decrement_trigger_(*joy_message) && !dribbler_decrement_trigger_(prev_joy_msg_)) {
      dribbler_speed_ -= get_parameter("dribbler_speed_step").as_double();
    }
    dribbler_speed_ =
      std::clamp(
      dribbler_speed_, static_cast<float>(get_parameter("mapping.dribbler.min").as_double()),
      static_cast<float>(get_parameter("mapping.dribbler.max").as_double()));
    if (dribbler_spin_trigger_(*joy_message)) {
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

  TriggerFunction ParseTriggerFunction(const std::string & description)
  {
    std::stringstream stream(description);
    std::string token;
    std::vector<std::string> tokens;
    while (std::getline(stream, token, ' ')) {
      tokens.push_back(token);
    }
    return ParseTriggerFunction(tokens);
  }

  TriggerFunction ParseTriggerFunction(const std::span<std::string> tokens)
  {
    if(tokens.empty()) {
      return [](const sensor_msgs::msg::Joy &){return false;};
    }

    const auto & first_token = tokens.front();

    if(first_token == "not") {
      auto inner_func = ParseTriggerFunction(tokens);
      return [inner_func](const sensor_msgs::msg::Joy & m){return !inner_func(m);};
    }

    if(first_token == "button") {
      if(tokens.size() != 2) {
        throw std::runtime_error("Incorrect number of arguments to 'button' trigger.");
      }
      const auto & button_index_token = tokens[1];
      const auto button_index = std::stoi(button_index_token);
      return [button_index](const sensor_msgs::msg::Joy & m){
               return m.buttons.at(button_index);
             };
    }

    if(first_token == "axis") {
      if(tokens.size() != 4) {
        throw std::runtime_error("Incorrect number of arguments to 'axis' trigger.");
      }
      const auto & axis_index_token = tokens[1];
      const auto & operator_token = tokens[2];
      const auto & threshold_token = tokens[3];
      const auto axis_index = std::stoi(axis_index_token);
      const auto threshold = std::stod(threshold_token);
      if(operator_token.size() != 1) {
        throw std::runtime_error(std::string("Invalid operator: ") + operator_token);
      }
      switch(operator_token.at(0)) {
        case '<':
          return [axis_index, threshold](const sensor_msgs::msg::Joy & m){
                   return m.axes[axis_index] < threshold;
                 };
        case '>':
          return [axis_index, threshold](const sensor_msgs::msg::Joy & m) {
                   return m.axes[axis_index] > threshold;
                 };
        case '=':
          return [axis_index, threshold](const sensor_msgs::msg::Joy & m ) {
                   return std::abs(m.axes[axis_index] - threshold) < 1e-2;
                 };
        default:
          throw std::runtime_error(std::string("Invalid operator: ") + operator_token);
      }
    }

    throw std::runtime_error(std::string("Unrecognized trigger token: ") + first_token);
  }
};

}  // namespace ateam_joystick_control

RCLCPP_COMPONENTS_REGISTER_NODE(ateam_joystick_control::JoystickControlNode)
