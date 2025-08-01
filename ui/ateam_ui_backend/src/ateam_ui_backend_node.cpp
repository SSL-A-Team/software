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
#include <std_msgs/msg/string.hpp>

#include <boost/beast.hpp>
#include <boost/asio.hpp>
#include <nlohmann/json.hpp>

#include <set>
#include <chrono>
#include <mutex>
#include <memory>
#include <thread>
#include <optional>
#include <functional>

#include "message_conversions.hpp"

#include <ateam_common/game_controller_listener.hpp>
#include <ateam_common/topic_names.hpp>
#include <ateam_common/indexed_topic_helpers.hpp>

#include <ateam_msgs/msg/world.hpp>
#include <ateam_msgs/msg/ball_state.hpp>
#include <ateam_msgs/msg/robot_state.hpp>
#include <ateam_radio_msgs/msg/basic_telemetry.hpp>
#include <ateam_radio_msgs/msg/extended_telemetry.hpp>
#include <ateam_radio_msgs/msg/connection_status.hpp>
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

using ateam_ui_backend_node::message_conversions::fromMsg;
using ateam_common::indexed_topic_helpers::create_indexed_subscribers;

using namespace std::literals::chrono_literals;
namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace asio = boost::asio;
using tcp = asio::ip::tcp;

// ---------------------- WebSocket Session ----------------------
class WebSocketSession : public std::enable_shared_from_this<WebSocketSession> {
public:
  using Ptr = std::shared_ptr<WebSocketSession>;

  WebSocketSession(tcp::socket socket,
                  std::function<void(const std::string&)> on_msg_cb,
                  std::function<void(Ptr)> on_close_cb)
    : ws_(std::move(socket)),
    on_msg_cb_(std::move(on_msg_cb)),
    on_close_cb_(std::move(on_close_cb)) {}

  void start() {
    ws_.accept();
    do_read();
  }

  void send(const std::string& message) {
    if (ws_.is_open()) {
      beast::error_code ec;
      ws_.write(asio::buffer(message), ec);
      if (ec) {
        std::cerr << "WebSocket write failed: " << ec.message() << "\n";
      }
    }
  }

  void do_read() {
    ws_.async_read(buffer_, [self = shared_from_this()](beast::error_code ec, std::size_t) {
      if (ec) {
        self->on_close_cb_(self);
        return;
      }

      std::string msg = beast::buffers_to_string(self->buffer_.data());
      self->buffer_.consume(self->buffer_.size());

      self->on_msg_cb_(msg);
      self->do_read();
    });
  }

  void close() {
    if (ws_.is_open()) {
      beast::error_code ec;
      ws_.close(websocket::close_code::normal, ec);
      if (ec) {
        std::cerr << "WebSocket close error: " << ec.message() << "\n";
      }
    }
  }

  websocket::stream<tcp::socket> ws_;
  beast::flat_buffer buffer_;
  std::function<void(const std::string&)> on_msg_cb_;
  std::function<void(Ptr)> on_close_cb_;
};



class AteamUIBackendNode : public rclcpp::Node
{
public:
  AteamUIBackendNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("ateam_ui_backend_node", options),
      ioc_(1),
      acceptor_(ioc_),
      game_controller_listener_(*this)
  {

    reset_json_object();

    world_subscription_ =
      create_subscription<ateam_msgs::msg::World>(
      "kenobi_node/world",
      10,
      std::bind(&AteamUIBackendNode::world_callback, this, std::placeholders::_1));

    timer_ = create_wall_timer(10ms, std::bind(&AteamUIBackendNode::send_latest_message, this));

    latest_msg_ = "default message";

    // WebSocket Acceptor Setup
    tcp::endpoint endpoint(tcp::v4(), 9001);
    acceptor_.open(endpoint.protocol());
    acceptor_.set_option(asio::socket_base::reuse_address(true));
    acceptor_.bind(endpoint);
    acceptor_.listen();
    do_accept();

    // Start IO thread
    ws_thread_ = std::thread([this]() {
        ioc_.run();
    });

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

  ~AteamUIBackendNode() override {
      RCLCPP_INFO(get_logger(), "SHUTTING DOWN");
      shutdown();
      ioc_.stop();
      if (ws_thread_.joinable()) {
          ws_thread_.join();
      }
  }

private:
  nlohmann::json json_;
  std::optional<std::string> latest_msg_;
  std::mutex json_mutex_;

  // ROS 2
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Boost.Asio / WebSocket
  asio::io_context ioc_;
  tcp::acceptor acceptor_;
  std::thread ws_thread_;

  // Sessions
  std::set<std::shared_ptr<WebSocketSession>> sessions_;
  std::mutex sessions_mutex_;

  ateam_common::GameControllerListener game_controller_listener_;

  std::array<rclcpp::Subscription<ateam_msgs::msg::RobotState>::SharedPtr,
    16> blue_robots_subscriptions_;
  std::array<rclcpp::Subscription<ateam_msgs::msg::RobotState>::SharedPtr,
    16> yellow_robots_subscriptions_;
  std::array<rclcpp::Subscription<ateam_radio_msgs::msg::BasicTelemetry>::SharedPtr,
    16> basic_telemetry_subscriptions_;
  // std::array<rclcpp::Subscription<ateam_radio_msgs::msg::ExtendedTelemetry>::SharedPtr,
  //   16> extended_telemetry_subscriptions_;
  std::array<rclcpp::Subscription<ateam_radio_msgs::msg::ConnectionStatus>::SharedPtr,
    16> connection_status_subscriptions_;

  rclcpp::Subscription<ateam_msgs::msg::World>::SharedPtr world_subscription_;

  void do_accept() {
    acceptor_.async_accept([this](beast::error_code ec, tcp::socket socket) {
      if (!ec) {
        auto session = std::make_shared<WebSocketSession>(
          std::move(socket),
          [this](const std::string& msg) { handle_incoming_message(msg); },
          [this](std::shared_ptr<WebSocketSession> s) { remove_session(s); });

        {
          std::lock_guard<std::mutex> lock(sessions_mutex_);
          sessions_.insert(session);
          RCLCPP_INFO(get_logger(), "adding session, total: '%ld'", sessions_.size());
        }

        session->start();
      }
      do_accept();  // Accept next client
    });
  }

  void send_latest_message() {

    const auto our_color = game_controller_listener_.GetTeamColor();
    std::string color;

    switch(our_color) {
      case ateam_common::TeamColor::Blue:
        color = "blue";
        break;
      case ateam_common::TeamColor::Yellow:
        color = "yellow";
        break;
      default:
        color = "unkown";
        break;
    }

    nlohmann::json json_copy;
    {
      std::lock_guard<std::mutex> lock(json_mutex_);
      json_["team"] = color;
      json_copy = json_;
    }

    std::string json_dump = json_copy.dump();
    std::lock_guard<std::mutex> lock(sessions_mutex_);
    for (auto& session : sessions_) {
      session->send(json_dump);
    }
  }

  void handle_incoming_message(const std::string &msg) {
    RCLCPP_INFO(get_logger(), "Received from WS client: '%s'", msg.c_str());
  }

  void remove_session(std::shared_ptr<WebSocketSession> session) {
    std::lock_guard<std::mutex> lock(sessions_mutex_);
    sessions_.erase(session);
    RCLCPP_INFO(get_logger(), "Closed session, remaining: '%ld'", sessions_.size());
  }

  void shutdown() {
    {
      std::lock_guard<std::mutex> lock(sessions_mutex_);
      for (auto& session : sessions_) {
        session->close();
      }
      sessions_.clear();
    }

    beast::error_code ec;
    acceptor_.close(ec);
    if (ec) {
      RCLCPP_WARN(get_logger(), "Error closing acceptor: %s", ec.message().c_str());
    }
  }

  void world_callback(
    const ateam_msgs::msg::World & world_msg)
  {
    std::lock_guard<std::mutex> lock(json_mutex_);
    json_ = fromMsg(world_msg);
  }

  /*
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
    json_ = {
        {"teamName", "A-Team"},
        {"team", "unkown"},
        {"teams", {}},
        {"ball", nullptr},
        {"field", nullptr},
        {"referee", nullptr},
        {"ai", {
          {"name", "No Play Specified"},
          {"description", "{'No Play Specified': ''}"}
        }},
        {"timestamp", 0}
      };

    // Fill robot array
    for (std::string color : {"blue", "yellow"}) {
      for (int i = 0; i < 16; ++i) {
        auto robot_json = message_conversions::fromMsg(ateam_msgs::msg::RobotState());
        robot_json["id"] = i;
        robot_json["team"] = color;
        robot_json["visible"] = false;
        robot_json["status"] = message_conversions::fromMsg(ateam_radio_msgs::msg::BasicTelemetry());

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