#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ateam_msgs/msg/robot_state.hpp>
#include <ateam_msgs/msg/ball_state.hpp>
#include <ateam_msgs/msg/robot_motion_command.hpp>
#include <ssl_league_msgs/msg/referee.hpp>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <eigen3/Eigen/Dense>

namespace ateam_dummy_ai
{

class PDController
{
public:
  double kP = 1.0;
  double kD = 0.1;

  double Step(const double error) {
    double effort = (kP * error) + (kD * (error - prev_error));
    prev_error = error;
    return effort;
  }

private:
  double prev_error = 0.0;
};

class FollowBallNode : public rclcpp::Node
{
public:
  explicit FollowBallNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("follow_ball_node", options),
    state_message_synchronizer_(robot_subscription_, ball_subscription_, 10)
  {
    offset = declare_parameter<double>("offset", offset);
    max_speed = declare_parameter<double>("max_speed", max_speed);
    controller.kP = declare_parameter<double>("controller.kP", controller.kP);
    controller.kD = declare_parameter<double>("controller.kD", controller.kD);
    on_set_params_callback_handle_ = add_on_set_parameters_callback(std::bind(&FollowBallNode::OnParametersSet, this, std::placeholders::_1));
    motion_command_publisher_ = create_publisher<ateam_msgs::msg::RobotMotionCommand>(
      "~/motion_command", rclcpp::SystemDefaultsQoS());
    robot_subscription_.subscribe(this, "~/robot_state", rclcpp::SystemDefaultsQoS().get_rmw_qos_profile());
    ball_subscription_.subscribe(this, "~/ball_state", rclcpp::SystemDefaultsQoS().get_rmw_qos_profile());
    state_message_synchronizer_.registerCallback(&FollowBallNode::StateCallback, this);
    referee_subscription_ = create_subscription<ssl_league_msgs::msg::Referee>("~/referee", rclcpp::SystemDefaultsQoS(), std::bind(&FollowBallNode::RefereeCallback, this, std::placeholders::_1));
  }

private:
  rclcpp::Publisher<ateam_msgs::msg::RobotMotionCommand>::SharedPtr motion_command_publisher_;
  message_filters::Subscriber<ateam_msgs::msg::RobotState> robot_subscription_;
  message_filters::Subscriber<ateam_msgs::msg::BallState> ball_subscription_;
  message_filters::TimeSynchronizer<ateam_msgs::msg::RobotState, ateam_msgs::msg::BallState> state_message_synchronizer_;
  rclcpp::Subscription<ssl_league_msgs::msg::Referee>::SharedPtr referee_subscription_;
  OnSetParametersCallbackHandle::SharedPtr on_set_params_callback_handle_;
  double offset = 200.0;
  double max_speed = 1.0;
  PDController controller;
  ssl_league_msgs::msg::Referee::_command_type referee_command = ssl_league_msgs::msg::Referee::COMMAND_HALT;

  rcl_interfaces::msg::SetParametersResult OnParametersSet(const std::vector<rclcpp::Parameter> & parameters) {
    for(const auto& parameter : parameters) {
      if(parameter.get_name() == "controller.kP") {
        controller.kP = parameter.as_double();
      }
      else if(parameter.get_name() == "controller.kD") {
        controller.kD = parameter.as_double();
      }
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  void StateCallback(const ateam_msgs::msg::RobotState::SharedPtr robot_state, const ateam_msgs::msg::BallState::SharedPtr ball_state)
  {
    if(referee_command == ssl_league_msgs::msg::Referee::COMMAND_HALT) {
      ateam_msgs::msg::RobotMotionCommand motion_command;
      motion_command.twist.linear.x = 0.0;
      motion_command.twist.linear.y = 0.0;
      motion_command_publisher_->publish(motion_command);
      return;
    }
    const auto min_offset = referee_command == ssl_league_msgs::msg::Referee::COMMAND_STOP ? 500.0 : 0.0;
    const Eigen::Vector2d ball_vec{ball_state->pose.position.x - robot_state->pose.position.x, ball_state->pose.position.y - robot_state->pose.position.y};
    const auto error = ball_vec.norm() - std::max(offset, min_offset);
    const auto speed = std::clamp(controller.Step(error), -max_speed, max_speed);
    const Eigen::Vector2d velocity = ball_vec.normalized() * speed;

    ateam_msgs::msg::RobotMotionCommand motion_command;
    motion_command.twist.linear.x = velocity.x();
    motion_command.twist.linear.y = velocity.y();
    motion_command_publisher_->publish(motion_command);
  }

  void RefereeCallback(const ssl_league_msgs::msg::Referee::SharedPtr ref_msg) {
    referee_command = ref_msg->command;
  }

};

}  // namespace ateam_dummy_ai

RCLCPP_COMPONENTS_REGISTER_NODE(ateam_dummy_ai::FollowBallNode)
