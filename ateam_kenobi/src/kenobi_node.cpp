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


#include <tf2/convert.h>
#include <tf2/utils.h>
#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ateam_msgs/msg/ball_state.hpp>
#include <ateam_radio_msgs/msg/basic_telemetry.hpp>
#include <ateam_radio_msgs/msg/connection_status.hpp>
#include <ateam_msgs/msg/robot_state.hpp>
#include <ateam_msgs/msg/field_info.hpp>
#include <ateam_msgs/msg/robot_motion_command.hpp>
#include <ateam_msgs/msg/overlay.hpp>
#include <ateam_msgs/msg/play_info.hpp>
#include <ateam_msgs/msg/world.hpp>
#include <ateam_msgs/srv/set_override_play.hpp>
#include <ateam_msgs/srv/set_play_enabled.hpp>
#include <ateam_msgs/msg/playbook_state.hpp>
#include <ateam_common/cache_directory.hpp>
#include <ateam_common/game_controller_listener.hpp>
#include <ateam_common/topic_names.hpp>
#include <ateam_common/indexed_topic_helpers.hpp>
#include <ateam_geometry/types.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "core/types/world.hpp"
#include "core/types/message_conversions.hpp"
#include "core/play_selector.hpp"
#include "core/in_play_eval.hpp"
#include "core/double_touch_eval.hpp"
#include "core/ballsense_emulator.hpp"
#include "core/ballsense_filter.hpp"
#include "core/motion/world_to_body_vel.hpp"
#include "plays/halt_play.hpp"
#include "core/defense_area_enforcement.hpp"
#include "core/joystick_enforcer.hpp"
#include <ateam_spatial/spatial_evaluator.hpp>

namespace ateam_kenobi
{

using ateam_common::indexed_topic_helpers::create_indexed_publishers;
using ateam_common::indexed_topic_helpers::create_indexed_subscribers;
using namespace std::literals::chrono_literals;

class KenobiNode : public rclcpp::Node
{
public:
  explicit KenobiNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("kenobi_node", options),
    play_selector_(*this),
    joystick_enforcer_(*this),
    overlays_(""),
    game_controller_listener_(*this)
  {
    world_.spatial_evaluator = &spatial_evaluator_;

    initialize_robot_ids();

    declare_parameter<bool>("use_world_velocities", false);
    declare_parameter<bool>("use_emulated_ballsense", false);

    overlay_publisher_ = create_publisher<ateam_msgs::msg::OverlayArray>(
      "/overlays",
      rclcpp::SystemDefaultsQoS());

    play_info_publisher_ = create_publisher<ateam_msgs::msg::PlayInfo>(
      "/play_info",
      rclcpp::SystemDefaultsQoS());

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

    create_indexed_subscribers<ateam_radio_msgs::msg::BasicTelemetry>(
      robot_feedback_subscriptions_,
      Topics::kRobotFeedbackPrefix,
      10,
      &KenobiNode::robot_feedback_callback,
      this);

    create_indexed_subscribers<ateam_radio_msgs::msg::ConnectionStatus>(
      robot_connection_status_subscriptions_,
      Topics::kRobotConnectionStatusPrefix,
      10,
      &KenobiNode::robot_connection_callback,
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

    field_subscription_ = create_subscription<ateam_msgs::msg::FieldInfo>(
      std::string(Topics::kField),
      10,
      std::bind(&KenobiNode::field_callback, this, std::placeholders::_1));

    override_service_ = create_service<ateam_msgs::srv::SetOverridePlay>(
      "~/set_override_play", std::bind(
        &KenobiNode::set_override_play_callback, this, std::placeholders::_1,
        std::placeholders::_2));

    play_enabled_service_ = create_service<ateam_msgs::srv::SetPlayEnabled>(
      "~/set_play_enabled", std::bind(
        &KenobiNode::set_play_enabled_callback, this, std::placeholders::_1,
        std::placeholders::_2));

    playbook_state_publisher_ = create_publisher<ateam_msgs::msg::PlaybookState>(
      "~/playbook_state",
      rclcpp::SystemDefaultsQoS());

    timer_ = create_wall_timer(10ms, std::bind(&KenobiNode::timer_callback, this));

    const auto playbook_path = declare_parameter<std::string>("playbook", "");
    const auto autosave_playbook_path = getCacheDirectory() / "playbook/autosave.json";
    if(!playbook_path.empty()) {
      RCLCPP_INFO(get_logger(), "Loading playbook from %s", playbook_path.c_str());
      play_selector_.loadFromFile(playbook_path);
    } else if(std::filesystem::exists(autosave_playbook_path)) {
      RCLCPP_INFO(get_logger(), "Loading playbook from autosave.");
      play_selector_.loadFromFile(autosave_playbook_path);
    } else {
      RCLCPP_INFO(get_logger(), "Using default playbook.");
    }

    RCLCPP_INFO(get_logger(), "Kenobi node ready.");
  }

  ~KenobiNode()
  {
    RCLCPP_INFO(get_logger(), "Autosaving playbook");
    play_selector_.saveToFile(getCacheDirectory() / "playbook/autosave.json");
  }

private:
  ateam_spatial::SpatialEvaluator spatial_evaluator_;
  World world_;
  PlaySelector play_selector_;
  InPlayEval in_play_eval_;
  DoubleTouchEval double_touch_eval_;
  BallSenseEmulator ballsense_emulator_;
  BallSenseFilter ballsense_filter_;
  std::vector<uint8_t> heatmap_render_buffer_;
  JoystickEnforcer joystick_enforcer_;
  visualization::Overlays overlays_;
  rclcpp::Publisher<ateam_msgs::msg::OverlayArray>::SharedPtr overlay_publisher_;
  rclcpp::Publisher<ateam_msgs::msg::PlayInfo>::SharedPtr play_info_publisher_;
  rclcpp::Subscription<ateam_msgs::msg::BallState>::SharedPtr ball_subscription_;
  std::array<rclcpp::Subscription<ateam_msgs::msg::RobotState>::SharedPtr,
    16> blue_robots_subscriptions_;
  std::array<rclcpp::Subscription<ateam_msgs::msg::RobotState>::SharedPtr,
    16> yellow_robots_subscriptions_;
  std::array<rclcpp::Subscription<ateam_radio_msgs::msg::BasicTelemetry>::SharedPtr,
    16> robot_feedback_subscriptions_;
  std::array<rclcpp::Subscription<ateam_radio_msgs::msg::ConnectionStatus>::SharedPtr,
    16> robot_connection_status_subscriptions_;
  rclcpp::Subscription<ateam_msgs::msg::FieldInfo>::SharedPtr
    field_subscription_;
  std::array<rclcpp::Publisher<ateam_msgs::msg::RobotMotionCommand>::SharedPtr,
    16> robot_commands_publishers_;
  rclcpp::Service<ateam_msgs::srv::SetOverridePlay>::SharedPtr override_service_;
  rclcpp::Service<ateam_msgs::srv::SetPlayEnabled>::SharedPtr play_enabled_service_;
  rclcpp::Publisher<ateam_msgs::msg::PlaybookState>::SharedPtr playbook_state_publisher_;

  ateam_common::GameControllerListener game_controller_listener_;

  rclcpp::Publisher<ateam_msgs::msg::World>::SharedPtr world_publisher_;

  rclcpp::TimerBase::SharedPtr timer_;

  void initialize_robot_ids()
  {
    for(auto i = 0u; i < world_.our_robots.size(); ++i) {
      world_.our_robots[i].id = i;
    }
    for(auto i = 0u; i < world_.their_robots.size(); ++i) {
      world_.their_robots[i].id = i;
    }
  }

  void blue_robot_state_callback(
    const ateam_msgs::msg::RobotState::SharedPtr robot_state_msg,
    int id)
  {
    const auto our_color = game_controller_listener_.GetTeamColor();
    if(our_color == ateam_common::TeamColor::Unknown) {
      return;
    }
    const auto are_we_blue = our_color == ateam_common::TeamColor::Blue;
    auto & robot_state_array = are_we_blue ? world_.our_robots : world_.their_robots;
    robot_state_callback(robot_state_array, id, robot_state_msg);
  }

  void yellow_robot_state_callback(
    const ateam_msgs::msg::RobotState::SharedPtr robot_state_msg,
    int id)
  {
    const auto our_color = game_controller_listener_.GetTeamColor();
    if(our_color == ateam_common::TeamColor::Unknown) {
      return;
    }
    const auto are_we_yellow = our_color == ateam_common::TeamColor::Yellow;
    auto & robot_state_array = are_we_yellow ? world_.our_robots : world_.their_robots;
    robot_state_callback(robot_state_array, id, robot_state_msg);
  }

  void robot_state_callback(
    std::array<Robot, 16> & robot_states,
    std::size_t id,
    const ateam_msgs::msg::RobotState::SharedPtr robot_state_msg)
  {
    robot_states.at(id).visible = robot_state_msg->visible;
    if (robot_state_msg->visible) {
      robot_states.at(id).pos = ateam_geometry::Point(
        robot_state_msg->pose.position.x,
        robot_state_msg->pose.position.y);
      tf2::Quaternion tf2_quat;
      tf2::fromMsg(robot_state_msg->pose.orientation, tf2_quat);
      robot_states.at(id).theta = tf2::getYaw(tf2_quat);
      robot_states.at(id).vel = ateam_geometry::Vector(
        robot_state_msg->twist.linear.x,
        robot_state_msg->twist.linear.y);
      robot_states.at(id).omega = robot_state_msg->twist.angular.z;
      robot_states.at(id).id = id;
    }
  }

  void robot_feedback_callback(
    const ateam_radio_msgs::msg::BasicTelemetry::SharedPtr robot_feedback_msg,
    int id)
  {
    world_.our_robots.at(id).breakbeam_ball_detected = robot_feedback_msg->breakbeam_ball_detected;
    world_.our_robots.at(id).kicker_available = robot_feedback_msg->kicker_available;
    world_.our_robots.at(id).chipper_available = robot_feedback_msg->chipper_available;
  }

  void robot_connection_callback(
    const ateam_radio_msgs::msg::ConnectionStatus::SharedPtr robot_connection_msg,
    int id)
  {
    world_.our_robots.at(id).radio_connected = robot_connection_msg->radio_connected;
  }

  void ball_state_callback(const ateam_msgs::msg::BallState::SharedPtr ball_state_msg)
  {
    world_.ball.visible = ball_state_msg->visible;
    if (ball_state_msg->visible) {
      world_.ball.pos = ateam_geometry::Point(
        ball_state_msg->pose.position.x,
        ball_state_msg->pose.position.y);
      world_.ball.vel = ateam_geometry::Vector(
        ball_state_msg->twist.linear.x,
        ball_state_msg->twist.linear.y);
    }
  }

  void field_callback(const ateam_msgs::msg::FieldInfo::SharedPtr field_msg)
  {
    Field field;
    field.field_length = field_msg->field_length;
    field.field_width = field_msg->field_width;
    field.goal_width = field_msg->goal_width;
    field.goal_depth = field_msg->goal_depth;
    field.boundary_width = field_msg->boundary_width;
    field.defense_area_depth = field_msg->defense_area_depth;
    field.defense_area_width = field_msg->defense_area_width;

    auto convert_point_array = [&](auto & starting_array, auto final_array_iter) {
        std::transform(
          starting_array.begin(), starting_array.end(), final_array_iter,
          [&](auto & val) {
            return ateam_geometry::Point(val.x, val.y);
          });
      };

    convert_point_array(field_msg->field_corners.points, field.field_corners.begin());
    convert_point_array(
      field_msg->ours.defense_area_corners.points,
      field.ours.defense_area_corners.begin());
    convert_point_array(field_msg->ours.goal_corners.points, field.ours.goal_corners.begin());
    convert_point_array(
      field_msg->theirs.defense_area_corners.points,
      field.theirs.defense_area_corners.begin());
    convert_point_array(field_msg->theirs.goal_corners.points, field.theirs.goal_corners.begin());
    field.ours.goal_posts = {field.ours.goal_corners[0], field.ours.goal_corners[1]};
    field.theirs.goal_posts = {field.theirs.goal_corners[0], field.theirs.goal_corners[1]};

    world_.field = field;
    if (game_controller_listener_.GetTeamSide() == ateam_common::TeamSide::PositiveHalf) {
      world_.ignore_side = -field_msg->ignore_side;
    } else {
      world_.ignore_side = field_msg->ignore_side;
    }
  }

  void set_override_play_callback(
    const ateam_msgs::srv::SetOverridePlay::Request::SharedPtr request,
    ateam_msgs::srv::SetOverridePlay::Response::SharedPtr response)
  {
    if (!request->play_name.empty() &&
      play_selector_.getPlayByName(request->play_name) == nullptr)
    {
      response->success = false;
      response->reason = "No such play.";
      return;
    }

    play_selector_.setPlayOverride(request->play_name);
    response->success = true;
  }

  void set_play_enabled_callback(
    const ateam_msgs::srv::SetPlayEnabled::Request::SharedPtr request,
    ateam_msgs::srv::SetPlayEnabled::Response::SharedPtr response)
  {
    auto play = play_selector_.getPlayByName(request->play_name);
    if (play == nullptr) {
      response->success = false;
      response->reason = "No such play.";
      return;
    }

    if (!request->enabled && dynamic_cast<plays::HaltPlay *>(play) != nullptr) {
      response->success = false;
      response->reason = "You can't disable the Halt play.";
      return;
    }

    play->setEnabled(request->enabled);
    response->success = true;
  }

  void timer_callback()
  {
    world_.current_time = std::chrono::steady_clock::now();
    world_.referee_info.running_command = game_controller_listener_.GetGameCommand();
    const auto & ref_msg = game_controller_listener_.GetLatestRefereeMessage();
    world_.referee_info.command_time =
      std::chrono::system_clock::time_point(
      std::chrono::nanoseconds(
        rclcpp::Time(ref_msg.command_timestamp).nanoseconds()));
    world_.referee_info.prev_command = game_controller_listener_.GetPreviousGameCommand();
    world_.referee_info.next_command = game_controller_listener_.GetNextGameCommand();
    world_.referee_info.current_game_stage = game_controller_listener_.GetGameStage();

    const auto & gc_designated_position =
      game_controller_listener_.GetDesignatedPosition();
    if (gc_designated_position.has_value()) {
      if (game_controller_listener_.GetTeamSide() == ateam_common::TeamSide::PositiveHalf) {
        world_.referee_info.designated_position = ateam_geometry::Point(
        -gc_designated_position->x,
        -gc_designated_position->y);
      } else {
        world_.referee_info.designated_position = ateam_geometry::Point(
        gc_designated_position->x,
        gc_designated_position->y);
      }
    } else {
      world_.referee_info.designated_position = std::nullopt;
    }


    if (game_controller_listener_.GetOurGoalieID().has_value()) {
      world_.referee_info.our_goalie_id = game_controller_listener_.GetOurGoalieID().value();
    }
    if (game_controller_listener_.GetTheirGoalieID().has_value()) {
      world_.referee_info.their_goalie_id = game_controller_listener_.GetTheirGoalieID().value();
    }
    in_play_eval_.Update(world_);
    double_touch_eval_.update(world_, overlays_);
    UpdateSpatialEvaluator();
    if (get_parameter("use_emulated_ballsense").as_bool()) {
      ballsense_emulator_.Update(world_);
    }
    ballsense_filter_.Update(world_);
    if (game_controller_listener_.GetTeamColor() == ateam_common::TeamColor::Unknown) {
      auto & clk = *this->get_clock();
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), clk, 3000,
        "DETECTED TEAM COLOR WAS UNKNOWN");
    }
    if (game_controller_listener_.GetTeamSide() == ateam_common::TeamSide::Unknown) {
      auto & clk = *this->get_clock();
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), clk, 3000,
        "DETECTED TEAM SIDE WAS UNKNOWN");
    }

    world_publisher_->publish(ateam_kenobi::message_conversions::toMsg(world_));

    auto motion_commands = runPlayFrame(world_);

    defense_area_enforcement::EnforceDefenseAreaKeepout(world_, motion_commands);

    joystick_enforcer_.RemoveCommandForJoystickBot(motion_commands);

    if (get_parameter("use_world_velocities").as_bool()) {
      motion::ConvertBodyVelsToWorldVels(motion_commands, world_.our_robots);
    } else {
      motion::ConvertWorldVelsToBodyVels(motion_commands, world_.our_robots);
    }

    send_all_motion_commands(motion_commands);
  }

  std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> runPlayFrame(
    const World & world)
  {
    ateam_msgs::msg::PlaybookState playbook_state;
    stp::Play * play = play_selector_.getPlay(world, playbook_state);
    playbook_state_publisher_->publish(playbook_state);

    if (play == nullptr) {
      RCLCPP_ERROR(get_logger(), "No play selected!");
      return {};
    }
    const auto motion_commands = play->runFrame(world);

    overlays_.merge(play->getOverlays());

    overlay_publisher_->publish(overlays_.getMsg());
    overlays_.clear();
    play->getOverlays().clear();


    ateam_msgs::msg::PlayInfo play_info_msg;
    play_info_msg.name = play->getName();
    play_info_msg.description = play->getPlayInfo().dump();
    play->getPlayInfo().clear();
    play_info_publisher_->publish(play_info_msg);

    return motion_commands;
  }

  void send_all_motion_commands(
    const std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>,
    16> & robot_motion_commands)
  {
    for (std::size_t id = 0; id < robot_commands_publishers_.size(); id++) {
      const auto & maybe_motion_command = robot_motion_commands.at(id);
      if (maybe_motion_command.has_value()) {
        robot_commands_publishers_.at(id)->publish(maybe_motion_command.value());
        world_.our_robots.at(id).prev_command_vel = ateam_geometry::Vector(
          maybe_motion_command.value().twist.linear.x,
          maybe_motion_command.value().twist.linear.y
        );

        world_.our_robots.at(id).prev_command_omega = maybe_motion_command.value().twist.angular.z;
      } else {
        world_.our_robots.at(id).prev_command_vel = ateam_geometry::Vector(0, 0);
        world_.our_robots.at(id).prev_command_omega = 0;
      }
    }
  }

  std::filesystem::path getCacheDirectory()
  {
    return ateam_common::getCacheDirectory() / "kenobi";
  }

  void UpdateSpatialEvaluator()
  {
    ateam_spatial::FieldDimensions field{
      world_.field.field_width,
      world_.field.field_length,
      world_.field.goal_width,
      world_.field.goal_depth,
      world_.field.boundary_width,
      world_.field.defense_area_width,
      world_.field.defense_area_depth
    };
    ateam_spatial::Ball ball{
      static_cast<float>(world_.ball.pos.x()),
      static_cast<float>(world_.ball.pos.y()),
      static_cast<float>(world_.ball.vel.x()),
      static_cast<float>(world_.ball.vel.y())
    };
    auto make_spatial_robot = [](const Robot & robot){
        return ateam_spatial::Robot{
        robot.visible,
        static_cast<float>(robot.pos.x()),
        static_cast<float>(robot.pos.y()),
        static_cast<float>(robot.theta),
        static_cast<float>(robot.vel.x()),
        static_cast<float>(robot.vel.y()),
        static_cast<float>(robot.omega)
        };
      };
    std::array<ateam_spatial::Robot, 16> our_robots;
    std::ranges::transform(world_.our_robots, our_robots.begin(), make_spatial_robot);
    std::array<ateam_spatial::Robot, 16> their_robots;
    std::ranges::transform(world_.their_robots, their_robots.begin(), make_spatial_robot);
    spatial_evaluator_.UpdateMaps(field, ball, our_robots, their_robots);
  }
};

}  // namespace ateam_kenobi

RCLCPP_COMPONENTS_REGISTER_NODE(ateam_kenobi::KenobiNode)
