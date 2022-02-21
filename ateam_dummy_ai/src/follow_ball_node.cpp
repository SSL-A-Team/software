#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ateam_msgs/msg/robot_state.hpp>
#include <ateam_msgs/msg/ball_state.hpp>
#include <ateam_msgs/msg/robot_motion_command.hpp>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>

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
    x_offset = declare_parameter<double>("x_offset", x_offset);
    y_offset = declare_parameter<double>("y_offset", y_offset);
    x_controller.kP = declare_parameter<double>("controllers.x.kP", x_controller.kP);
    x_controller.kD = declare_parameter<double>("controllers.x.kD", x_controller.kD);
    y_controller.kP = declare_parameter<double>("controllers.y.kP", y_controller.kP);
    y_controller.kD = declare_parameter<double>("controllers.y.kD", y_controller.kD);
    motion_command_publisher_ = create_publisher<ateam_msgs::msg::RobotMotionCommand>(
      "~/motion_command", rclcpp::SystemDefaultsQoS());
    robot_subscription_.subscribe(this, "~/robot_state", rclcpp::SystemDefaultsQoS().get_rmw_qos_profile());
    ball_subscription_.subscribe(this, "~/ball_state", rclcpp::SystemDefaultsQoS().get_rmw_qos_profile());
    state_message_synchronizer_.registerCallback(&FollowBallNode::StateCallback, this);
  }

private:
  rclcpp::Publisher<ateam_msgs::msg::RobotMotionCommand>::SharedPtr motion_command_publisher_;
  message_filters::Subscriber<ateam_msgs::msg::RobotState> robot_subscription_;
  message_filters::Subscriber<ateam_msgs::msg::BallState> ball_subscription_;
  message_filters::TimeSynchronizer<ateam_msgs::msg::RobotState, ateam_msgs::msg::BallState> state_message_synchronizer_;
  double x_offset = 200.0;
  double y_offset = 0.0;
  double max_vel = 1.0;
  PDController x_controller;
  PDController y_controller;

  void StateCallback(const ateam_msgs::msg::RobotState::SharedPtr robot_state, const ateam_msgs::msg::BallState::SharedPtr ball_state)
  {
    const auto x_diff = ball_state->pose.position.x - robot_state->pose.position.x + x_offset;
    const auto y_diff = ball_state->pose.position.y - robot_state->pose.position.y + y_offset;
    ateam_msgs::msg::RobotMotionCommand motion_command;
    motion_command.twist.linear.x = std::clamp(x_controller.Step(x_diff), -max_vel, max_vel);
    motion_command.twist.linear.y = std::clamp(y_controller.Step(y_diff), -max_vel, max_vel);
    motion_command_publisher_->publish(motion_command);
  }

};

}  // namespace ateam_dummy_ai

RCLCPP_COMPONENTS_REGISTER_NODE(ateam_dummy_ai::FollowBallNode)
