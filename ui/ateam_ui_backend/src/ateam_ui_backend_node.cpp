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
#include "rclcpp/serialization.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include "rosbag2_transport/reader_writer_factory.hpp"

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

#include <rcl_interfaces/srv/set_parameters.hpp>
#include <ssl_ros_bridge_msgs/srv/set_desired_keeper.hpp>
#include <ateam_msgs/srv/set_play_enabled.hpp>
#include <ateam_msgs/srv/set_override_play.hpp>
#include <ateam_msgs/srv/set_ignore_field_side.hpp>
#include <ateam_radio_msgs/srv/send_robot_power_request.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <ateam_msgs/srv/send_simulator_control_packet.hpp>
#include <ssl_league_msgs/msg/simulator_control.hpp>
#include <ssl_league_msgs/msg/teleport_ball_command.hpp>
#include <ssl_league_msgs/msg/teleport_robot_command.hpp>

#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace ateam_ui_backend_node
{

using namespace ateam_ui_backend_node::message_conversions;
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

  void send(const std::vector<uint8_t> bytes) {
    if (ws_.is_open()) {
      ws_.binary(true);
      beast::error_code ec;
      ws_.write(asio::buffer(bytes.data(), bytes.size()), ec);
      if (ec) {
        std::cerr << "WebSocket write failed: " << ec.message() << "\n";
      }
    }
  }

  void send(const std::string& message) {
    if (ws_.is_open()) {
      ws_.binary(false);
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

    enable_subscribers();

    /*
    ball_subscription_ =
      create_subscription<ateam_msgs::msg::BallState>(
      "/ball",
      10,
      std::bind(&AteamUIBackendNode::ball_state_callback, this, std::placeholders::_1));

    create_indexed_subscribers<ateam_msgs::msg::RobotState>(
      blue_robots_subscriptions_,
      Topics::kBlueTeamRobotPrefix,
      10,
      &AteamUIBackendNode::blue_robot_state_callback,
      this);

    create_indexed_subscribers<ateam_msgs::msg::RobotState>(
      yellow_robots_subscriptions_,
      Topics::kYellowTeamRobotPrefix,
      10,
      &AteamUIBackendNode::yellow_robot_state_callback,
      this);

    create_indexed_subscribers<ateam_msgs::msg::RobotFeedback>(
      robot_feedback_subscriptions_,
      Topics::kRobotFeedbackPrefix,
      10,
      &AteamUIBackendNode::robot_feedback_callback,
      this);
    */

    // Params
    // declare_parameter("joystick_param", -1);
    declare_parameter<int>("/joystick_control_node:robot_id", -1);
    // joystick_param_ = rclcpp::Parameter("joystick_param", -1);

    // Services
    set_joystick_robot_client_ =
      create_client<rcl_interfaces::srv::SetParameters>("/joystick_control_node/set_parameters");
    set_desired_keeper_client_ =
      create_client<ssl_ros_bridge_msgs::srv::SetDesiredKeeper>("/team_client_node/set_desired_keeper");
    set_play_enabled_client_ =
      create_client<ateam_msgs::srv::SetPlayEnabled>("/kenobi_node/set_play_enabled");
    set_override_play_client_ =
      create_client<ateam_msgs::srv::SetOverridePlay>("/kenobi_node/set_override_play");
    set_ignore_field_side_client_ =
      create_client<ateam_msgs::srv::SetIgnoreFieldSide>("/field_manager/set_ignore_field_side");
    send_simulator_control_packet_client_ =
      create_client<ateam_msgs::srv::SendSimulatorControlPacket>("/radio_bridge/send_simulator_control_packet");
    send_robot_power_request_client_ =
      create_client<ateam_radio_msgs::srv::SendRobotPowerRequest>("/radio_bridge/send_robot_power_request");
    send_reboot_kenobi_request_client_ =
      create_client<std_srvs::srv::Trigger>("/strike_him_down");  

    // marker publisher
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("world_markers", 
      rclcpp::SystemDefaultsQoS());

    timer_ = create_wall_timer(10ms, std::bind(&AteamUIBackendNode::send_latest_message, this));
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

  std::vector<std::string> string_list_;

  bool shutting_down_ = false;
  bool loading_bag_file_ = false;

  // Boost.Asio / WebSocket
  asio::io_context ioc_;
  tcp::acceptor acceptor_;
  std::thread ws_thread_;

  // Sessions
  std::set<std::shared_ptr<WebSocketSession>> sessions_;
  std::mutex sessions_mutex_;

  // ROS 2
  rclcpp::TimerBase::SharedPtr timer_;

  // Team Name, Referee messages
  ateam_common::GameControllerListener game_controller_listener_;

  // Kenobi World and required subscriptions
  rclcpp::Subscription<ateam_msgs::msg::World>::SharedPtr world_subscription_;
  rclcpp::Subscription<ateam_msgs::msg::PlaybookState>::SharedPtr playbook_subscription_;
  rclcpp::Subscription<ssl_league_msgs::msg::Referee>::SharedPtr referee_subscription_;
  rclcpp::Subscription<ateam_msgs::msg::PlayInfo>::SharedPtr playinfo_subscription_;
  rclcpp::Subscription<ateam_msgs::msg::OverlayArray>::SharedPtr overlay_subscription_;
  rclcpp::Subscription<ateam_msgs::msg::JoystickControlStatus>::SharedPtr joystick_subscription_;

  // Alternate world status subscriptions
  rclcpp::Subscription<ateam_msgs::msg::BallState>::SharedPtr ball_subscription_;
  rclcpp::Subscription<ateam_msgs::msg::FieldInfo>::SharedPtr field_subscription_;

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

  // Services
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_joystick_robot_client_;
  rclcpp::Client<ssl_ros_bridge_msgs::srv::SetDesiredKeeper>::SharedPtr set_desired_keeper_client_;
  rclcpp::Client<ateam_msgs::srv::SetPlayEnabled>::SharedPtr set_play_enabled_client_;
  rclcpp::Client<ateam_msgs::srv::SetOverridePlay>::SharedPtr set_override_play_client_;
  rclcpp::Client<ateam_msgs::srv::SetIgnoreFieldSide>::SharedPtr set_ignore_field_side_client_;
  rclcpp::Client<ateam_msgs::srv::SendSimulatorControlPacket>::SharedPtr send_simulator_control_packet_client_;
  rclcpp::Client<ateam_radio_msgs::srv::SendRobotPowerRequest>::SharedPtr send_robot_power_request_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr send_reboot_kenobi_request_client_;

  // Publishers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  void enable_subscribers() {
    // Kenobi World and required Susbcriptions
    world_subscription_ =
      create_subscription<ateam_msgs::msg::World>(
      "kenobi_node/world",
      10,
      std::bind(&AteamUIBackendNode::world_callback, this, std::placeholders::_1));

    playbook_subscription_ =
      create_subscription<ateam_msgs::msg::PlaybookState>(
      "kenobi_node/playbook_state",
      10,
      std::bind(&AteamUIBackendNode::playbook_callback, this, std::placeholders::_1));

    referee_subscription_ =
      create_subscription<ssl_league_msgs::msg::Referee>(
      "/referee_messages",
      10,
      std::bind(&AteamUIBackendNode::referee_callback, this, std::placeholders::_1));

    playinfo_subscription_ =
      create_subscription<ateam_msgs::msg::PlayInfo>(
      "/play_info",
      10,
      std::bind(&AteamUIBackendNode::playinfo_callback, this, std::placeholders::_1));

    overlay_subscription_ =
      create_subscription<ateam_msgs::msg::OverlayArray>(
      "/overlays",
      10,
      std::bind(&AteamUIBackendNode::overlay_callback, this, std::placeholders::_1));


    // Alternate world status subscriptions

    field_subscription_ =
      create_subscription<ateam_msgs::msg::FieldInfo>(
      "/field",
      10,
      std::bind(&AteamUIBackendNode::field_callback, this, std::placeholders::_1));
  }

  void disable_subscribers() {
    world_subscription_.reset();
    playbook_subscription_.reset();
    referee_subscription_.reset();
    playinfo_subscription_.reset();
    overlay_subscription_.reset();
    field_subscription_.reset();
  }

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
      // load_bag("/home/mwoodward/ateam_ws/test_bag");
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
      if (json_["world"].is_null()) {
        return;
      }

      json_["world"]["team"] = color;

      json_copy = json_;

      json_["referee"] = nullptr;
      json_["play_info"] = nullptr;
      json_["overlays"] = nlohmann::json::array();
      json_["field"] = nullptr;
      json_["responses"] = nlohmann::json::array();
    }

    // std::string json_dump = json_copy.dump();
    std::vector<std::uint8_t> bytes = nlohmann::json::to_bson(json_copy);
    std::lock_guard<std::mutex> lock(sessions_mutex_);
    for (auto& session : sessions_) {
      // session->send(json_dump);
      session->send(bytes);
    }
  }

  void handle_incoming_message(const std::string &msg) {
    // RCLCPP_INFO(get_logger(), "Received from WS client: '%s'", msg.c_str());

    nlohmann::json json_msg = nlohmann::json::parse(msg);

    if (!json_msg.contains("msg_type")) {
      return;
    }
    MessageType msg_type = json_msg["msg_type"];
    switch(msg_type) {
      case MessageType::SetUseKenobiTopic:
        // RCLCPP_INFO(this->get_logger(), "CALLING setUseKenobiTopic");
        return;
      case MessageType::SetJoystickRobot:
        load_bag("/home/mwoodward/ateam_ws/test_bag");
        if (json_msg.contains("robot_id")) {
          auto request = toSetJoystickRobotRequest(json_msg);

          set_joystick_robot_client_->async_send_request(request,
            [&](rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture future) {
              // auto response = future.get();

              // std::lock_guard<std::mutex> lock(json_mutex_);
              // json_["responses"].push_back(fromSrvResponse(*response));
            }
          );
        }
        break;
      case MessageType::SetDesiredKeeper: {
        auto request = toSetDesiredKeeperRequest(json_msg);

        set_desired_keeper_client_->async_send_request(request,
          [&](rclcpp::Client<ssl_ros_bridge_msgs::srv::SetDesiredKeeper>::SharedFuture future) {
            auto response = future.get();

            std::lock_guard<std::mutex> lock(json_mutex_);
            json_["responses"].push_back(fromSrvResponse(*response));
          }
        );
        break;
      }
      case MessageType::SetPlayEnabled: {
        auto request = toSetPlayEnabledRequest(json_msg);

        set_play_enabled_client_->async_send_request(request,
          [&](rclcpp::Client<ateam_msgs::srv::SetPlayEnabled>::SharedFuture future) {
            auto response = future.get();

            std::lock_guard<std::mutex> lock(json_mutex_);
            json_["responses"].push_back(fromSrvResponse(*response));
          }
        );
        break;
      }
      case MessageType::SetOverridePlay: {
        auto request = toSetOverridePlayRequest(json_msg);

        set_override_play_client_->async_send_request(request,
          [&](rclcpp::Client<ateam_msgs::srv::SetOverridePlay>::SharedFuture future) {
            auto response = future.get();

            std::lock_guard<std::mutex> lock(json_mutex_);
            json_["responses"].push_back(fromSrvResponse(*response));
          }
        );
        break;
      }
      case MessageType::SetIgnoreFieldSide: {
        auto request = toIgnoreFieldSideRequest(json_msg);

        set_ignore_field_side_client_->async_send_request(request,
          [&](rclcpp::Client<ateam_msgs::srv::SetIgnoreFieldSide>::SharedFuture future) {
            auto response = future.get();

            std::lock_guard<std::mutex> lock(json_mutex_);
            json_["responses"].push_back(fromSrvResponse(*response));
          }
        );
        break;
      }
      case MessageType::SendPowerRequest: {
        auto request = toSendRobotPowerRequest(json_msg);

        send_robot_power_request_client_->async_send_request(request,
          [&](rclcpp::Client<ateam_radio_msgs::srv::SendRobotPowerRequest>::SharedFuture future) {
            auto response = future.get();

            std::lock_guard<std::mutex> lock(json_mutex_);
            json_["responses"].push_back(fromSrvResponse(*response));
          }
        );
        break;
      }
      case MessageType::SendRebootKenobiRequest: {
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

        send_reboot_kenobi_request_client_->async_send_request(request,
          [&](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
            auto response = future.get();

            std::lock_guard<std::mutex> lock(json_mutex_);
            json_["responses"].push_back(fromSrvResponse(*response));
          }
        );
        break;
      }
      case MessageType::SendSimulatorControlPacket: {
        auto request = toSendSimulatorControlPacketRequest(json_msg);

        send_simulator_control_packet_client_->async_send_request(request,
          [&](rclcpp::Client<ateam_msgs::srv::SendSimulatorControlPacket>::SharedFuture future) {
            auto response = future.get();

            std::lock_guard<std::mutex> lock(json_mutex_);
            json_["responses"].push_back(fromSrvResponse(*response));
          }
        );
        break;
      }
      default:
        // Invalid message type
        RCLCPP_INFO(get_logger(), "Received Invalid Message Type: '%d'", msg_type);
        return;
    }
  }

  void remove_session(std::shared_ptr<WebSocketSession> session) {
    std::lock_guard<std::mutex> lock(sessions_mutex_);
    sessions_.erase(session);
    RCLCPP_INFO(get_logger(), "Closed session, remaining: '%ld'", sessions_.size());
  }

  void shutdown() {
    {
      shutting_down_ = true;
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
    json_["world"] = fromMsg(world_msg);
    // json_["world"] = world_msg;
  }

  void playbook_callback(
    const ateam_msgs::msg::PlaybookState & playbook_msg)
  {
    std::lock_guard<std::mutex> lock(json_mutex_);
    json_["playbook"] = fromMsg(playbook_msg);
  }

  void referee_callback(
    const ssl_league_msgs::msg::Referee & referee_msg)
  {
    std::lock_guard<std::mutex> lock(json_mutex_);
    json_["referee"] = fromMsg(referee_msg);
  }

  void playinfo_callback(
    const ateam_msgs::msg::PlayInfo & play_info_msg)
  {
    std::lock_guard<std::mutex> lock(json_mutex_);
    json_["play_info"] = fromMsg(play_info_msg);
  }

  void overlay_callback(
    const ateam_msgs::msg::OverlayArray & overlay_msg)
  {
    std::lock_guard<std::mutex> lock(json_mutex_);
    json_["overlays"].push_back(fromMsg(overlay_msg));
  }

  void field_callback(
    const ateam_msgs::msg::FieldInfo & field_msg)
  {
    std::lock_guard<std::mutex> lock(json_mutex_);
    json_["field"] = fromMsg(field_msg);
  }

  void blue_robot_state_callback(
    const ateam_msgs::msg::RobotState robot_state_msg,
    int id)
  {
    std::lock_guard<std::mutex> lock(json_mutex_);

    const auto our_color = game_controller_listener_.GetTeamColor();
    if (our_color == ateam_common::TeamColor::Blue) {
      json_["world"]["our_team"][id] = fromMsg(robot_state_msg);
    } else if (our_color == ateam_common::TeamColor::Yellow) {
      json_["world"]["their_team"][id] = fromMsg(robot_state_msg);
    }
  }

  void yellow_robot_state_callback(
    const ateam_msgs::msg::RobotState robot_state_msg,
    int id)
  {
    std::lock_guard<std::mutex> lock(json_mutex_);

    const auto our_color = game_controller_listener_.GetTeamColor();
    if (our_color == ateam_common::TeamColor::Yellow) {
      json_["world"]["our_team"][id] = fromMsg(robot_state_msg);
    } else if (our_color == ateam_common::TeamColor::Blue) {
      json_["world"]["their_team"][id] = fromMsg(robot_state_msg);
    }
  }

  void ball_state_callback(const ateam_msgs::msg::BallState ball_state_msg)
  {
    std::lock_guard<std::mutex> lock(json_mutex_);
    json_["world"]["ball"] = fromMsg(ball_state_msg);
  }

  void load_bag(std::string bag_path) {
    disable_subscribers();
    timer_->cancel();
    loading_bag_file_ = true;

    RCLCPP_INFO(get_logger(), "START LOADING BAG: '%s'", bag_path.c_str());

    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_path;

    const auto world_serialization = rclcpp::Serialization<ateam_msgs::msg::World>();
    const auto playbook_serialization = rclcpp::Serialization<ateam_msgs::msg::PlaybookState>();
    const auto referee_serialization = rclcpp::Serialization<ssl_league_msgs::msg::Referee>();
    const auto playinfo_serialization = rclcpp::Serialization<ateam_msgs::msg::PlayInfo>();
    const auto overlay_serialization = rclcpp::Serialization<ateam_msgs::msg::OverlayArray>();
    const auto field_serialization = rclcpp::Serialization<ateam_msgs::msg::FieldInfo>();

    const auto reader = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options);

    reader->open(storage_options);

    const auto metadata = reader->get_metadata();
    const double bag_length_s = std::round(metadata.duration.count() / (1e9));
    const auto print_duration = std::chrono::milliseconds(1000);

    auto time = std::chrono::steady_clock::now();
    auto load_start_time = time;
    long long bag_start_time = metadata.starting_time.time_since_epoch().count();
    double prev_print_time = 0.0;
    double prev_recv_time = 0.0;

    int counter = 0;
    while (reader->has_next()) {
      if (shutting_down_) {
        break;
      }

      rosbag2_storage::SerializedBagMessageSharedPtr msg = reader->read_next();
      if (msg->topic_name == "/kenobi_node/world") {
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        ateam_msgs::msg::World::SharedPtr world_msg = std::make_shared<ateam_msgs::msg::World>();
        world_serialization.deserialize_message(&serialized_msg, world_msg.get());

        world_callback(*world_msg);
        send_latest_message();
        // Trigger UI message send
        // if (counter >= 1499) {
        //   send_latest_message();
        //   counter = 0;
        // } else {
        //   counter++;
        // }
      } else if (msg->topic_name == "/kenobi_node/playbook_state") {
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        ateam_msgs::msg::PlaybookState::SharedPtr playbook_msg = std::make_shared<ateam_msgs::msg::PlaybookState>();
        playbook_serialization.deserialize_message(&serialized_msg, playbook_msg.get());

        // playbook_callback(*playbook_msg);
      } else if (msg->topic_name == "/referee_messages") {
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        ssl_league_msgs::msg::Referee::SharedPtr referee_msg = std::make_shared<ssl_league_msgs::msg::Referee>();
        referee_serialization.deserialize_message(&serialized_msg, referee_msg.get());

        // referee_callback(*referee_msg);
      } else if (msg->topic_name == "/play_info") {
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        ateam_msgs::msg::PlayInfo::SharedPtr playinfo_msg = std::make_shared<ateam_msgs::msg::PlayInfo>();
        playinfo_serialization.deserialize_message(&serialized_msg, playinfo_msg.get());

        // playinfo_callback(*playinfo_msg);
      } else if (msg->topic_name == "/overlays") {
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        ateam_msgs::msg::OverlayArray::SharedPtr overlay_msg = std::make_shared<ateam_msgs::msg::OverlayArray>();
        overlay_serialization.deserialize_message(&serialized_msg, overlay_msg.get());

        // overlay_callback(*overlay_msg);
      } else if (msg->topic_name == "/field") {
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        ateam_msgs::msg::FieldInfo::SharedPtr field_msg = std::make_shared<ateam_msgs::msg::FieldInfo>();
        field_serialization.deserialize_message(&serialized_msg, field_msg.get());

        // field_callback(*field_msg);
      }

      if (std::chrono::steady_clock::now() - time > print_duration) {
        double bag_time_since_start = (msg->recv_timestamp - bag_start_time) / (1e9);
        double bag_time_since_last_print = bag_time_since_start - prev_print_time;
        // This assumes we are printing once every second

        RCLCPP_INFO(get_logger(), "LOADING BAG: %.2f / %f: %.2fx realtime",
          (msg->recv_timestamp - bag_start_time) / (1e9),
          bag_length_s,
          bag_time_since_last_print
        );
        RCLCPP_INFO(get_logger(), "string list length: %ld", string_list_.size());

        prev_print_time = bag_time_since_start;
        time = std::chrono::steady_clock::now();
      }
      prev_recv_time = (msg->recv_timestamp - bag_start_time) / (1e9);
    }

    if (!shutting_down_) {
      RCLCPP_INFO(get_logger(), "FINISHED LOADING BAG: '%s'", bag_path.c_str());
    }

    double total_load_time = (std::chrono::steady_clock::now() - load_start_time).count() / (1e9);
    RCLCPP_INFO(get_logger(), "Took %.2fs to load %fs: %.2fx realtime",
      total_load_time,
      prev_recv_time,
      prev_recv_time / total_load_time
    );

    /*
    RCLCPP_INFO(get_logger(), "final packet size: %ld", string_list_.back().size());
    RCLCPP_INFO(get_logger(), "%s", string_list_.back().c_str());
    long int total_size = 0;
    for (std::string str : string_list_) {
      total_size += str.size();
    }
    RCLCPP_INFO(get_logger(), "total combined packet size: %ld", total_size);
    */



    loading_bag_file_ = false;
    enable_subscribers();
  }

  void publish_marker_array(const ateam_msgs::msg::World & world_msg) {

    auto arr = visualization_msgs::msg::MarkerArray();

    auto ball = visualization_msgs::msg::Marker();
    ball.ns = "ball";
    ball.id = 1;
    ball.type = 2; // sphere

    ball.pose = world_msg.balls[0].pose;
    ball.scale.x = 0.042;
    ball.scale.y = 0.042;
    ball.scale.z = 0.042;

    ball.color.r = 1.0;
    ball.color.g = 0.36;
    ball.color.b = 0.0;
    ball.color.a = 1.0;

    arr.markers.push_back(ball);

    const auto our_color = game_controller_listener_.GetTeamColor();

    for (int i = 0; i < 16; ++i) {
      const auto robot = world_msg.our_robots.at(i);

      if (robot.visible) {
        auto marker = visualization_msgs::msg::Marker();

        marker.ns = "our_robots";
        marker.id = i;
        marker.type = 3; // cylinder

        marker.pose = robot.pose;
        marker.scale.x = 0.18;
        marker.scale.y = 0.18;
        marker.scale.z = 0.15;

        if (our_color == ateam_common::TeamColor::Blue) {
          marker.color.r = 0.0;
          marker.color.g = 0.0;
          marker.color.b = 1.0;
          marker.color.a = 1.0;
        } else {
          marker.color.r = 1.0;
          marker.color.g = 1.0;
          marker.color.b = 0.0;
          marker.color.a = 1.0;
        }

        arr.markers.push_back(marker);
      }
    }

    for (int i = 0; i < 16; ++i) {
      const auto robot = world_msg.their_robots.at(i);

      if (robot.visible) {
        auto marker = visualization_msgs::msg::Marker();

        marker.ns = "their_robots";
        marker.id = i;
        marker.type = 3; // cylinder

        marker.pose = robot.pose;
        marker.scale.x = 0.18;
        marker.scale.y = 0.18;
        marker.scale.z = 0.15;

        if (our_color == ateam_common::TeamColor::Yellow) {
          marker.color.r = 0.0;
          marker.color.g = 0.0;
          marker.color.b = 1.0;
          marker.color.a = 1.0;
        } else {
          marker.color.r = 1.0;
          marker.color.g = 1.0;
          marker.color.b = 0.0;
          marker.color.a = 1.0;
        }

        arr.markers.push_back(marker);
      }
    }
    marker_pub_->publish(arr);
  }

  void reset_json_object() {
    json_ = {
      {"world", nullptr},
      {"playbook", nullptr},
      {"referee", nullptr},
      {"play_info", nullptr},
      {"overlays", nlohmann::json::array()},
      {"field", nullptr},
      {"responses", nlohmann::json::array()}
    };
  }
};
} // namespace ateam_ui_backend_node

RCLCPP_COMPONENTS_REGISTER_NODE(ateam_ui_backend_node::AteamUIBackendNode)