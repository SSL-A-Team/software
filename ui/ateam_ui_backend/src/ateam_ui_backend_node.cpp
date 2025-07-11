// Copyright 2025 A Team
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

#include <nlohmann/json.hpp>

#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio.hpp>
#include <iostream>
#include <thread>
#include <chrono>
#include <mutex>

#include <ateam_common/game_controller_listener.hpp>
#include <ateam_common/topic_names.hpp>
#include <ateam_common/indexed_topic_helpers.hpp>

#include "message_conversions.hpp"

#include <ateam_msgs/msg/world.hpp>
#include <ateam_msgs/msg/ball_state.hpp>
#include <ateam_msgs/msg/robot_state.hpp>
#include <ateam_msgs/msg/robot_feedback.hpp>
#include <ateam_msgs/msg/behavior_executor_state.hpp>
#include <ateam_msgs/msg/overlay_array.hpp>
#include <ateam_msgs/msg/field_info.hpp>
#include <ssl_league_msgs/msg/referee.hpp>
#include <ateam_msgs/msg/play_info.hpp>
#include <ateam_msgs/msg/playbook_state.hpp>
#include <ateam_msgs/msg/joystick_control_status.hpp>

#include <ateam_msgs/srv/set_desired_keeper.hpp>
#include <ateam_msgs/srv/set_play_enabled.hpp>
#include <ateam_msgs/srv/set_override_play.hpp>
#include <ateam_msgs/srv/set_ignore_field_side.hpp>

#include <ateam_msgs/srv/send_simulator_control_packet.hpp>
#include <ssl_league_msgs/msg/simulator_control.hpp>
#include <ssl_league_msgs/msg/teleport_ball_command.hpp>
#include <ssl_league_msgs/msg/teleport_robot_command.hpp>


namespace ateam_ui_backend_node
{

using ateam_common::indexed_topic_helpers::create_indexed_subscribers;

using namespace std::literals::chrono_literals;
namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = net::ip::tcp;

class AteamUIBackendNode : public rclcpp::Node
{
public:
  explicit AteamUIBackendNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : rclcpp::Node("ateam_ui_backend_node", options),
      io_context_(), acceptor_{io_context_, {tcp::v4(), 9001}},
      game_controller_listener_(*this)
  {

    reset_json_object();

    /*
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

    create_indexed_subscribers<ateam_msgs::msg::RobotFeedback>(
      robot_feedback_subscriptions_,
      Topics::kRobotFeedbackPrefix,
      10,
      &KenobiNode::robot_feedback_callback,
      this);
    */
  }

  //const std::lock_guard<std::mutex> lock(world_mutex_);

private:
  nlohmann::json json_;
  std::mutex json_mutex_;

  net::io_context io_context_;
  tcp::acceptor acceptor_;

  ateam_common::GameControllerListener game_controller_listener_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::array<rclcpp::Subscription<ateam_msgs::msg::RobotState>::SharedPtr,
    16> blue_robots_subscriptions_;
  std::array<rclcpp::Subscription<ateam_msgs::msg::RobotState>::SharedPtr,
    16> yellow_robots_subscriptions_;
  std::array<rclcpp::Subscription<ateam_msgs::msg::RobotFeedback>::SharedPtr,
    16> robot_feedback_subscriptions_;


  void poll_new_connections() {
    std::cout<<"polling new connections"<<std::endl;
    tcp::socket socket{io_context_};
    acceptor_.accept(socket);
    std::thread([this, s = std::move(socket)]() mutable
                { this->do_session(std::move(s)); })
        .detach();
  }

  void do_session(tcp::socket socket)
  {
    try
    {
      websocket::stream<tcp::socket> ws(std::move(socket));
      ws.accept();

      while (true)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        ws.write(net::buffer("Data every 10ms"));
      }
    }
    catch (std::exception const &e)
    {
      std::cerr << "Error: " << e.what() << "\n";
    }
  }

  /*
  void world_callback(
    const ateam_msgs::msg::World::SharedPtr msg)
  {
    const auto our_color = game_controller_listener_.GetTeamColor();

    // TODO: Handle this better
    if(our_color == ateam_common::TeamColor::Unknown) {
      return;
    }

    const auto are_we_blue = our_color == ateam_common::TeamColor::Blue;
  }

  void blue_robot_state_callback(
    const ateam_msgs::msg::RobotState::SharedPtr robot_state_msg,
    int id)
  {
    std::lock_guard<std::mutex> lock(json_mutex_);
  }

  void yellow_robot_state_callback(
    const ateam_msgs::msg::RobotState::SharedPtr robot_state_msg,
    int id)
  {
    std::lock_guard<std::mutex> lock(json_mutex_);
  }

  void ball_state_callback(const ateam_msgs::msg::BallState::SharedPtr ball_state_msg)
  {
    std::lock_guard<std::mutex> lock(json_mutex_);
    json_["ball"] = fromMsg(ball_state_msg);
  }

  void field_callback(const ateam_msgs::msg::FieldInfo::SharedPtr field_msg)
  {
    std::lock_guard<std::mutex> lock(json_mutex_);
    json_["field"] = fromMsg(field_msg);
  }

  void referee_callback(const ssl_league_msgs::msg::Referee::SharedPtr referee_msg)
  {
    std::lock_guard<std::mutex> lock(json_mutex_);
    json_["referee"] = fromMsg(referee_msg);
  }

  void ai_state_callback(const ateam_msgs::msg::PlayInfo::SharedPtr ai_state_msg)
  {
    std::lock_guard<std::mutex> lock(json_mutex_);
    json_["ai"] = fromMsg(ai_state_msg);
  }


  */

  void reset_json_object() {
    // Mimics a default WorldState typescript object
    std::string Data{R"(
      {
        "teamName": "A-Team",
        "team": "unkown",
        "teams": [],
        "ball": null,
        "field": null,
        "referee": null,
        "ai": {
          "name": "No Play Specified",
          "description": "{'No Play Specified':''}",
        },
        "timestamp": 0
      }
    )"};

    json_ = nlohmann::json::parse(Data);

    // Fill robot array
    for (std::string color : {"blue", "yellow"}) {
      for (int i = 0; i < 16; ++i) {
        auto robot_json = message_conversions::fromMsg(ateam_msgs::msg::RobotState());
        robot_json["id"] = i;
        robot_json["team"] = color;
        robot_json["visible"] = false;
        robot_json["status"] = message_conversions::fromMsg(ateam_msgs::msg::RobotFeedback());

        json_["teams"][color][i] = robot_json;
      }
    }

    json_["ball"] = message_conversions::fromMsg(ateam_msgs::msg::BallState());
    json_["field"] = message_conversions::fromMsg(ateam_msgs::msg::FieldInfo());
    json_["referee"] = message_conversions::fromMsg(ssl_league_msgs::msg::Referee());

    auto timestamp = rclcpp::Time(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());

    json_["timestamp"] = message_conversions::fromMsg(timestamp);
  }
};
} // namespace ateam_ui_backend_node

RCLCPP_COMPONENTS_REGISTER_NODE(ateam_ui_backend_node::AteamUIBackendNode)