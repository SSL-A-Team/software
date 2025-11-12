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
#include <ateam_msgs/msg/robot_motion_command.hpp>
#include <ateam_msgs/msg/overlay.hpp>
#include <ateam_msgs/msg/play_info.hpp>
#include <ateam_msgs/msg/game_state_world.hpp>
#include <ateam_msgs/srv/set_override_play.hpp>
#include <ateam_msgs/srv/set_play_enabled.hpp>
#include <ateam_msgs/msg/playbook_state.hpp>
#include <ateam_msgs/msg/kenobi_status.hpp>
#include <ateam_common/cache_directory.hpp>
#include <ateam_common/game_controller_listener.hpp>
#include <ateam_common/topic_names.hpp>
#include <ateam_common/indexed_topic_helpers.hpp>
#include <ateam_game_state/type_adapters.hpp>
#include <ateam_geometry/types.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "core/types/state_types.hpp"
#include "core/play_selector.hpp"
#include "core/in_play_eval.hpp"
#include "core/double_touch_eval.hpp"
#include "core/ballsense_emulator.hpp"
#include "core/ballsense_filter.hpp"
#include "core/motion/frame_conversions.hpp"
#include "core/motion/motion_executor.hpp"
#include "plays/halt_play.hpp"
#include "core/defense_area_enforcement.hpp"
#include "core/joystick_enforcer.hpp"
#include "core/fps_tracker.hpp"

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
    motion_executor_(get_logger().get_child("motion"))
  {
    declare_parameter<bool>("use_world_velocities", false);
    declare_parameter<bool>("use_emulated_ballsense", false);

    overlay_publisher_ = create_publisher<ateam_msgs::msg::OverlayArray>(
      "/overlays",
      rclcpp::SystemDefaultsQoS());

    play_info_publisher_ = create_publisher<ateam_msgs::msg::PlayInfo>(
      "/play_info",
      rclcpp::SystemDefaultsQoS());

    create_indexed_publishers<ateam_msgs::msg::RobotMotionCommand>(
      robot_commands_publishers_, Topics::kRobotMotionCommandPrefix,
      rclcpp::SystemDefaultsQoS(), this);

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

    status_publisher_ = create_publisher<ateam_msgs::msg::KenobiStatus>("~/status",
        rclcpp::SystemDefaultsQoS());

    world_subscription_ = create_subscription<World>(
      std::string(Topics::kWorld), rclcpp::SystemDefaultsQoS(),
      std::bind(&KenobiNode::WorldCallback, this, std::placeholders::_1));

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
  PlaySelector play_selector_;
  InPlayEval in_play_eval_;
  DoubleTouchEval double_touch_eval_;
  BallSenseEmulator ballsense_emulator_;
  BallSenseFilter ballsense_filter_;
  std::vector<uint8_t> heatmap_render_buffer_;
  JoystickEnforcer joystick_enforcer_;
  visualization::Overlays overlays_;
  FpsTracker fps_tracker_;
  motion::MotionExecutor motion_executor_;
  rclcpp::Publisher<ateam_msgs::msg::OverlayArray>::SharedPtr overlay_publisher_;
  rclcpp::Publisher<ateam_msgs::msg::PlayInfo>::SharedPtr play_info_publisher_;
  std::array<rclcpp::Publisher<ateam_msgs::msg::RobotMotionCommand>::SharedPtr,
    16> robot_commands_publishers_;
  rclcpp::Service<ateam_msgs::srv::SetOverridePlay>::SharedPtr override_service_;
  rclcpp::Service<ateam_msgs::srv::SetPlayEnabled>::SharedPtr play_enabled_service_;
  rclcpp::Publisher<ateam_msgs::msg::PlaybookState>::SharedPtr playbook_state_publisher_;
  rclcpp::Publisher<ateam_msgs::msg::KenobiStatus>::SharedPtr status_publisher_;
  rclcpp::Subscription<World>::SharedPtr world_subscription_;

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

  void WorldCallback(const ateam_game_state::World & world)
  {
    ateam_msgs::msg::KenobiStatus status_msg;
    status_msg.fps = fps_tracker_.Update(world);
    status_publisher_->publish(status_msg);

    auto motion_commands = runPlayFrame(world);

    defense_area_enforcement::EnforceDefenseAreaKeepout(world, motion_commands);

    joystick_enforcer_.RemoveCommandForJoystickBot(motion_commands);

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

    const auto commands = play->runFrame(world);
    std::array<std::optional<motion::MotionIntent>, 16> motion_intents;
    std::ranges::transform(
      commands, motion_intents.begin(),
      [](const std::optional<RobotCommand> & cmd) -> std::optional<motion::MotionIntent> {
        if (cmd.has_value()) {
          return cmd->motion_intent;
        } else {
          return std::nullopt;
        }
      });
    const auto motion_commands = motion_executor_.RunFrame(motion_intents, overlays_, world);

    const auto use_world_vels = get_parameter("use_world_velocities").as_bool();

    std::array<std::optional<ateam_msgs::msg::RobotMotionCommand>, 16> ros_commands;
    for(auto id = 0ul; id < commands.size(); ++id) {
      auto & maybe_cmd = commands[id];
      auto & maybe_motion_cmd = motion_commands[id];
      if (!maybe_cmd || !maybe_motion_cmd) {
        ros_commands[id] = std::nullopt;
      } else {
        const auto & robot = world.our_robots[id];
        const auto & cmd = maybe_cmd.value();
        const auto & motion_cmd = maybe_motion_cmd.value();
        const auto linear_vel = use_world_vels ?
          motion::LocalToWorldFrame(motion_cmd.linear, robot) : motion_cmd.linear;
        auto & ros_cmd = ros_commands[id].emplace();
        ros_cmd.dribbler_speed = cmd.dribbler_speed;
        ros_cmd.kick_request = static_cast<uint8_t>(cmd.kick);
        ros_cmd.kick_speed = cmd.kick_speed;
        ros_cmd.twist.linear.x = linear_vel.x();
        ros_cmd.twist.linear.y = linear_vel.y();
        ros_cmd.twist.angular.z = motion_cmd.angular;
        ros_cmd.twist_frame =
          use_world_vels ? ateam_msgs::msg::RobotMotionCommand::FRAME_WORLD :
          ateam_msgs::msg::RobotMotionCommand::FRAME_BODY;
      }
    }

    overlays_.merge(play->getOverlays());
    overlay_publisher_->publish(overlays_.getMsg());
    overlays_.clear();
    play->getOverlays().clear();


    ateam_msgs::msg::PlayInfo play_info_msg;
    play_info_msg.name = play->getName();
    play_info_msg.description = play->getPlayInfo().dump();
    play->getPlayInfo().clear();
    play_info_publisher_->publish(play_info_msg);

    return ros_commands;
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

  std::filesystem::path getCacheDirectory()
  {
    return ateam_common::getCacheDirectory() / "kenobi";
  }
};

}  // namespace ateam_kenobi

RCLCPP_COMPONENTS_REGISTER_NODE(ateam_kenobi::KenobiNode)
