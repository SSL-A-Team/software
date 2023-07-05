#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ateam_msgs/msg/ball_state.hpp>
#include <ateam_msgs/msg/robot_state.hpp>
#include <ateam_msgs/msg/field_info.hpp>
#include <ateam_msgs/msg/robot_motion_command.hpp>
#include <ateam_msgs/msg/overlay.hpp>
#include <ateam_msgs/msg/world.hpp>
#include <ateam_common/team_info_listener.hpp>
#include <ateam_common/game_state_listener.hpp>
#include <ateam_common/topic_names.hpp>
#include <ateam_common/indexed_topic_helpers.hpp>
#include <ateam_geometry/types.hpp>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "types/world.hpp"
#include "types/message_conversions.hpp"
#include "play_selector.hpp"

namespace ateam_kenobi
{

using namespace ateam_common::indexed_topic_helpers;
using namespace std::literals::chrono_literals;

class KenobiNode : public rclcpp::Node
{
public:
  explicit KenobiNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("kenobi_node", options),
    info_listener_(*this),
    game_state_listener_(*this)
  {
    create_indexed_subscribers<ateam_msgs::msg::RobotState>(
      blue_robots_subscriptions_,
      Topics::kBlueTeamRobotPrefix,
      10,
      &KenobiNode::blue_robot_state_callback,
      this);

    create_indexed_subscribers<ateam_msgs::msg::RobotState>(
      yellow_robots_subscriptions_,
      Topics::kYellowTeamRobotPrefix,
      10,
      &KenobiNode::yellow_robot_state_callback,
      this);

    create_indexed_publishers<ateam_msgs::msg::RobotMotionCommand>(
      robot_commands_publishers_, Topics::kRobotMotionCommandPrefix,
      rclcpp::SystemDefaultsQoS(), this);

    ball_subscription_ = create_subscription<ateam_msgs::msg::BallState>(
      std::string(Topics::kBall),
      10,
      std::bind(&KenobiNode::ball_state_callback, this, std::placeholders::_1));

    world_publisher_ = create_publisher<ateam_msgs::msg::World>(
      "~/world",
      rclcpp::SystemDefaultsQoS());

    overlay_publisher_ = create_publisher<ateam_msgs::msg::Overlay>(
      "/overlay",
      rclcpp::SystemDefaultsQoS());
    // TODO overlay manager of some kind

    field_subscription_ = create_subscription<ateam_msgs::msg::FieldInfo>(
      std::string(Topics::kField),
      10,
      std::bind(&KenobiNode::field_callback, this, std::placeholders::_1));

    timer_ = create_wall_timer(10ms, std::bind(&KenobiNode::timer_callback, this));

    RCLCPP_INFO(get_logger(), "Kenobi node ready.");
  }

private:
  World world_;
  PlaySelector play_selector_;
  rclcpp::Subscription<ateam_msgs::msg::BallState>::SharedPtr ball_subscription_;
  std::array<rclcpp::Subscription<ateam_msgs::msg::RobotState>::SharedPtr,
    16> blue_robots_subscriptions_;
  std::array<rclcpp::Subscription<ateam_msgs::msg::RobotState>::SharedPtr,
    16> yellow_robots_subscriptions_;
  rclcpp::Subscription<ateam_msgs::msg::FieldInfo>::SharedPtr
    field_subscription_;
  std::array<rclcpp::Publisher<ateam_msgs::msg::RobotMotionCommand>::SharedPtr,
    16> robot_commands_publishers_;

  ateam_common::TeamInfoListener info_listener_;
  ateam_common::GameStateListener game_state_listener_;

  rclcpp::Publisher<ateam_msgs::msg::Overlay>::SharedPtr overlay_publisher_;
  rclcpp::Publisher<ateam_msgs::msg::World>::SharedPtr world_publisher_;

  rclcpp::TimerBase::SharedPtr timer_;

  void blue_robot_state_callback(
    const ateam_msgs::msg::RobotState::SharedPtr robot_state_msg,
    int id)
  {
    const auto are_we_blue = info_listener_.GetTeamColor() ==
      ateam_common::TeamInfoListener::TeamColor::Blue;
    auto & robot_state_array = are_we_blue ? world_.our_robots : world_.their_robots;
    robot_state_callback(robot_state_array, id, robot_state_msg);
  }

  void yellow_robot_state_callback(
    const ateam_msgs::msg::RobotState::SharedPtr robot_state_msg,
    int id)
  {
    const auto are_we_yellow = info_listener_.GetTeamColor() ==
      ateam_common::TeamInfoListener::TeamColor::Yellow;
    auto & robot_state_array = are_we_yellow ? world_.our_robots : world_.their_robots;
    robot_state_callback(robot_state_array, id, robot_state_msg);
  }

  void robot_state_callback(
    std::array<std::optional<Robot>, 16> & robot_states,
    std::size_t id,
    const ateam_msgs::msg::RobotState::SharedPtr robot_state_msg)
  {
    robot_states.at(id) = Robot();
    robot_states.at(id).value().pos = ateam_geometry::Point(robot_state_msg->pose.position.x, robot_state_msg->pose.position.y);
    tf2::Quaternion tf2_quat;
    tf2::fromMsg(robot_state_msg->pose.orientation, tf2_quat);
    robot_states.at(id).value().theta = tf2::getYaw(tf2_quat);
    robot_states.at(id).value().vel = ateam_geometry::Vector(robot_state_msg->twist.linear.x, robot_state_msg->twist.linear.y);
    robot_states.at(id).value().omega = robot_state_msg->twist.angular.z;
    robot_states.at(id).value().id = id;
  }

  void ball_state_callback(const ateam_msgs::msg::BallState::SharedPtr ball_state_msg)
  {
    world_.ball.pos = ateam_geometry::Point(ball_state_msg->pose.position.x, ball_state_msg->pose.position.y);
    world_.ball.vel = ateam_geometry::Vector(ball_state_msg->twist.linear.x, ball_state_msg->twist.linear.y);
  }

  void field_callback(const ateam_msgs::msg::FieldInfo::SharedPtr field_msg)
  {
    Field field;
    field.field_length = field_msg->field_length;
    field.field_width = field_msg->field_width;
    field.goal_width = field_msg->goal_width;
    field.goal_depth = field_msg->goal_depth;
    field.boundary_width = field_msg->boundary_width;

    auto convert_point_array = [&](auto & starting_array, auto final_array_iter) {
        std::transform(
          starting_array.begin(), starting_array.end(), final_array_iter,
          [&](auto & val) {
            return ateam_geometry::Point(val.x, val.y);
          });
      };

    convert_point_array(field_msg->field_corners, field.field_corners.begin());
    convert_point_array(field_msg->ours.goalie_corners, field.ours.goalie_corners.begin());
    convert_point_array(field_msg->ours.goal_posts, field.ours.goal_posts.begin());
    convert_point_array(field_msg->theirs.goalie_corners, field.theirs.goalie_corners.begin());
    convert_point_array(field_msg->theirs.goal_posts, field.theirs.goal_posts.begin());

    world_.field = field;
  }

  void timer_callback()
  {
    world_.current_time = std::chrono::steady_clock::now();

    world_.referee_info.running_command = game_state_listener_.GetGameCommand();
    world_.referee_info.current_game_stage = game_state_listener_.GetGameStage();

    world_publisher_->publish(ateam_kenobi::message_conversions::toMsg(world_));

    send_all_motion_commands(runPlayFrame(world_));
  }

  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> runPlayFrame(const World & world)
  {
    auto play = play_selector_.getPlay(world);
    return std::visit([this,&world](auto & play)->std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> {
      using PlayType = std::decay_t<decltype(play)>;
      if constexpr(std::is_same_v<PlayType, std::monostate>) {
        RCLCPP_ERROR(get_logger(), "No play selected!");
        return {};
      } else {
        return play.get().runFrame(world);
      }
    }, play);
  }

  void send_all_motion_commands(
    const std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
    16> & robot_motion_commands)
  {
    for (std::size_t id = 0; id < robot_commands_publishers_.size(); id++) {
      const auto & maybe_motion_command = robot_motion_commands.at(id);
      if (maybe_motion_command.has_value()) {
        robot_commands_publishers_.at(id)->publish(maybe_motion_command.value());
      }
    }
  }

};

}  // namespace ateam_kenobi

RCLCPP_COMPONENTS_REGISTER_NODE(ateam_kenobi::KenobiNode)
