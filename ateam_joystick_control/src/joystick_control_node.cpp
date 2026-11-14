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

#include <array>
#include <string>
#include <vector>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ateam_common/topic_names.hpp>
#include <ateam_common/robot_constants.hpp>
#include <ateam_msgs/msg/robot_motion_command.hpp>
#include <ateam_msgs/msg/joystick_control_status.hpp>
#include <ateam_radio_msgs/msg/extended_telemetry.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <cmath>
#include <optional>

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
    pivot_trigger_ = ParseTriggerFunction(declare_parameter<std::string>("mapping.pivot",
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
        {"dribbler.max", 1.0},
        {"dribbler.min", 0.0}
      });

    // controls the size of steps when changing dribbler speed
    declare_parameter<double>("dribbler_speed_step", 0.01);

    declare_parameter<double>("kick_speed", 5.0);
    declare_parameter<double>("chip_speed", 5.0);

    // Position mode: on entry (with vision active on the robot) the robot's estimated pose is
    // captured as the anchor, and the sticks command BCM_GLOBAL_POSITION targets offset from it:
    // x/y in the anchor's heading frame (stick forward = robot forward), scaled by
    // position_mode.linear_range (m), and theta by position_mode.angular_range (rad). Until the
    // anchor is captured the robot is held with a zero local-velocity command. The anchor is
    // re-captured whenever position mode is turned on or the controlled robot changes.
    declare_parameter<bool>("position_mode", false);

    // Trajectory acceleration limits sent with every command (m/s^2, rad/s^2);
    // 0 leaves the firmware default.
    declare_parameter<double>("max_accel_linear", 2.0);
    declare_parameter<double>("max_accel_angular", 10.0);
    declare_parameter<double>("position_mode.linear_range", 0.25);
    declare_parameter<double>("position_mode.angular_range", M_PI / 2);

    // Pivot (hold the pivot button): orbits the ball in front of the robot. The robot's
    // heading when the button is pressed is captured as the reference; the angular stick
    // then commands a target heading offset from it by up to pivot.angular_range (rad).
    // All pivot.* parameters are read every packet, so they can be tuned live
    // (e.g. rqt_reconfigure). 0 for max_angular_vel / max_angular_acc / orbit_radius
    // leaves the firmware default.
    declare_parameter<double>("pivot.angular_range", M_PI);
    // Angle (rad) between the orbit tangent and the robot heading, toward the orbit
    // center: 0 = robot faces along its direction of travel, pi/2 = faces the ball.
    declare_parameter<double>("pivot.inset_angle", 0.8,
      RangeDescriptor("rad, orbit tangent -> heading toward the ball (pi/2 = face the ball)",
      0.0, 1.6, 0.0));
    declare_parameter<bool>("pivot.compute_inset_angle", false);
    declare_parameter<bool>("pivot.backward", false);
    declare_parameter<double>("pivot.orbit_radius", 0.15,
      RangeDescriptor("m, distance from the robot center to the pivot point", 0.05, 0.3, 0.0));
    declare_parameter<double>("pivot.max_angular_vel", 0.0);
    declare_parameter<double>("pivot.max_angular_acc", 0.0);

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
  float dribbler_speed_ = kDefaultDribblerSetpoint;
  sensor_msgs::msg::Joy prev_joy_msg_;

  TriggerFunction kick_trigger_;
  TriggerFunction pivot_trigger_;
  TriggerFunction dribbler_increment_trigger_;
  TriggerFunction dribbler_decrement_trigger_;
  TriggerFunction dribbler_spin_trigger_;
  int linear_x_axis_;
  int linear_y_axis_;
  int angular_z_axis_;

  rclcpp::Publisher<ateam_msgs::msg::RobotMotionCommand>::SharedPtr control_publisher_;
  rclcpp::Publisher<ateam_msgs::msg::JoystickControlStatus>::SharedPtr status_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  rclcpp::Subscription<ateam_radio_msgs::msg::ExtendedTelemetry>::SharedPtr telemetry_subscription_;

  // Latest estimated pose of the controlled robot and when its firmware last applied vision.
  std::optional<std::array<double, 3>> robot_pose_;
  std::optional<rclcpp::Time> last_vision_time_;
  // Position-mode anchor pose (x, y, theta), captured on entry.
  std::optional<std::array<double, 3>> anchor_pose_;
  // Robot heading captured when the pivot button was pressed.
  std::optional<double> pivot_anchor_heading_;
  static constexpr double kVisionActiveTimeoutS = 0.5;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  static rcl_interfaces::msg::ParameterDescriptor RangeDescriptor(
    const std::string & description, double from, double to, double step)
  {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.description = description;
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = from;
    range.to_value = to;
    range.step = step;
    d.floating_point_range.push_back(range);
    return d;
  }

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
    telemetry_subscription_.reset();
    robot_pose_.reset();
    last_vision_time_.reset();
    anchor_pose_.reset();
    pivot_anchor_heading_.reset();
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
      telemetry_subscription_ = create_subscription<ateam_radio_msgs::msg::ExtendedTelemetry>(
        std::string(Topics::kRobotMotionFeedbackPrefix) + std::to_string(robot_id),
        rclcpp::SystemDefaultsQoS(),
        [this](const ateam_radio_msgs::msg::ExtendedTelemetry::SharedPtr msg) {
          const auto & bct = msg->body_control_telemetry;
          if (bct.kf_body_pos_estimate.size() >= 3) {
            robot_pose_ = {bct.kf_body_pos_estimate[0], bct.kf_body_pos_estimate[1],
              bct.kf_body_pos_estimate[2]};
          }
          if (bct.vision_update) {
            last_vision_time_ = now();
          }
        });
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

    const bool pivot_held = pivot_trigger_(*joy_message);
    if (!pivot_held) {
      pivot_anchor_heading_.reset();
    } else if (!pivot_anchor_heading_ && robot_pose_) {
      // Captured on the press (or as soon as the robot reports a pose while held).
      pivot_anchor_heading_ = (*robot_pose_)[2];
      RCLCPP_INFO(get_logger(), "Pivot anchored at heading %.3f rad", *pivot_anchor_heading_);
    }

    if (pivot_held && pivot_anchor_heading_) {
      command_message.body_control_mode = ateam_msgs::msg::RobotMotionCommand::BCM_HEADING_PIVOT;
      command_message.pivot_global_theta = std::remainder(
        *pivot_anchor_heading_ + get_parameter("pivot.angular_range").as_double() *
        joy_message->axes[angular_z_axis_], 2.0 * M_PI);
      command_message.pivot_orbit_radius = get_parameter("pivot.orbit_radius").as_double();
      command_message.pivot_inset_angle = get_parameter("pivot.inset_angle").as_double();
      command_message.pivot_compute_inset_angle =
        get_parameter("pivot.compute_inset_angle").as_bool();
      command_message.pivot_direction = get_parameter("pivot.backward").as_bool() ?
        ateam_msgs::msg::RobotMotionCommand::PD_BACKWARD :
        ateam_msgs::msg::RobotMotionCommand::PD_FORWARD;
      command_message.pivot_max_angular_vel = get_parameter("pivot.max_angular_vel").as_double();
      command_message.pivot_max_angular_acc = get_parameter("pivot.max_angular_acc").as_double();
    } else if (pivot_held) {
      // No pose from the robot yet to anchor the pivot: hold still.
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Pivot: waiting for a pose from the robot to anchor the heading");
      command_message.body_control_mode = ateam_msgs::msg::RobotMotionCommand::BCM_LOCAL_VELOCITY;
    } else if (get_parameter("position_mode").as_bool()) {
      if (!anchor_pose_ && robot_pose_ && VisionActive()) {
        anchor_pose_ = robot_pose_;
        RCLCPP_INFO(
          get_logger(), "Position mode anchored at (%.3f, %.3f, %.3f)",
          (*anchor_pose_)[0], (*anchor_pose_)[1], (*anchor_pose_)[2]);
      }
      if (anchor_pose_) {
        const auto [ax, ay, ath] = *anchor_pose_;
        const double range = get_parameter("position_mode.linear_range").as_double();
        const double dx = range * joy_message->axes[linear_x_axis_];
        const double dy = range * joy_message->axes[linear_y_axis_];
        command_message.body_control_mode =
          ateam_msgs::msg::RobotMotionCommand::BCM_GLOBAL_POSITION;
        command_message.pose.x = ax + std::cos(ath) * dx - std::sin(ath) * dy;
        command_message.pose.y = ay + std::sin(ath) * dx + std::cos(ath) * dy;
        command_message.pose.theta = std::remainder(
          ath + get_parameter("position_mode.angular_range").as_double() *
          joy_message->axes[angular_z_axis_], 2.0 * M_PI);
      } else {
        // Hold still until the robot reports vision and a pose to anchor to.
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Position mode: waiting for vision on the robot to anchor its start pose");
        command_message.body_control_mode =
          ateam_msgs::msg::RobotMotionCommand::BCM_LOCAL_VELOCITY;
      }
    } else {
      command_message.body_control_mode = ateam_msgs::msg::RobotMotionCommand::BCM_LOCAL_VELOCITY;
      command_message.velocity.x = get_parameter("mapping.linear.x.scale").as_double() *
        joy_message->axes[linear_x_axis_];
      command_message.velocity.y = get_parameter("mapping.linear.y.scale").as_double() *
        joy_message->axes[linear_y_axis_];
      command_message.velocity.theta = get_parameter("mapping.angular.z.scale").as_double() *
        joy_message->axes[angular_z_axis_];
    }

    command_message.limit_acc_linear = get_parameter("max_accel_linear").as_double();
    command_message.limit_acc_angular = get_parameter("max_accel_angular").as_double();

    if (kick_trigger_(*joy_message)) {
      command_message.kick_request = ateam_msgs::msg::RobotMotionCommand::KR_KICK_TOUCH;
      command_message.kick_speed = get_parameter("kick_speed").as_double();
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
      command_message.dribbler_mode = ateam_msgs::msg::RobotMotionCommand::DC_CURRENT;
      command_message.dribbler_setpoint = dribbler_speed_;
    } else {
      command_message.dribbler_mode = ateam_msgs::msg::RobotMotionCommand::DC_DISABLE;
      command_message.dribbler_setpoint = 0.0;
    }

    control_publisher_->publish(command_message);
    prev_joy_msg_ = *joy_message;
  }

  bool VisionActive()
  {
    return last_vision_time_ && (now() - *last_vision_time_).seconds() < kVisionActiveTimeoutS;
  }

  rcl_interfaces::msg::SetParametersResult SetParameterCallback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    for (const auto & param : parameters) {
      if (param.get_name() == "position_mode" && param.as_bool()) {
        // Re-anchor on every entry into position mode.
        anchor_pose_.reset();
      }
    }
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
