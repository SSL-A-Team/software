// Copyright 2024 A Team
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

#include "team_client.hpp"
#include <google/protobuf/util/delimited_message_util.h>
#include <string>

namespace ssl_ros_bridge::game_controller_bridge
{

TeamClient::TeamClient(rclcpp::Logger logger)
: logger_(logger),
  socket_(io_service_)
{
}

bool TeamClient::Connect(const ConnectionParameters & parameters)
{
  connected_ = false;
  if (!AttemptToConnectSocket(parameters.address, parameters.port)) {
    return false;
  }
  if (!AttemptToRegister(parameters.team_name, parameters.team_color)) {
    socket_.close();
    return false;
  }
  RCLCPP_INFO(logger_, "Team client connected to Game Controller!");
  connected_ = true;
  return true;
}

void TeamClient::Disconnect()
{
  connected_ = false;
  socket_.close();
}

TeamClient::Result TeamClient::SetDesiredKeeper(const int keeper_id)
{
  TeamToController team_to_controller;
  team_to_controller.set_desired_keeper(keeper_id);
  return SendRequest(team_to_controller);
}

TeamClient::Result TeamClient::RequestBotSubstitution()
{
  TeamToController team_to_controller;
  team_to_controller.set_substitute_bot(true);
  return SendRequest(team_to_controller);
}

TeamClient::Result TeamClient::SetAdvantageChoice(const AdvantageChoiceOption & choice)
{
  TeamToController team_to_controller;
  switch (choice) {
    case AdvantageChoiceOption::Stop:
      team_to_controller.set_advantage_choice(STOP);
      break;
    case AdvantageChoiceOption::Continue:
      team_to_controller.set_advantage_choice(CONTINUE);
      break;
  }
  return SendRequest(team_to_controller);
}

TeamClient::PingResult TeamClient::Ping()
{
  TeamToController team_to_controller;
  team_to_controller.set_ping(true);
  PingResult result;
  const auto send_time = std::chrono::steady_clock::now();
  result.request_result = SendRequest(team_to_controller);
  result.ping = std::chrono::steady_clock::now() - send_time;
  return result;
}

bool TeamClient::AttemptToConnectSocket(
  const boost::asio::ip::address & address,
  const uint16_t port)
{
  if (socket_.is_open()) {
    socket_.close();
  }
  boost::system::error_code error_code;
  RCLCPP_INFO(logger_, "Connecting to game controller at %s:%d", address.to_string().c_str(), port);
  socket_.connect(boost::asio::ip::tcp::endpoint(address, port), error_code);
  if (error_code) {
    RCLCPP_WARN(logger_, "Team client connect failed: %s", error_code.message().c_str());
    return false;
  }

  ControllerToTeam controller_to_team;
  if (!WaitForReply(controller_to_team)) {
    return false;
  }

  if (!controller_to_team.has_controller_reply()) {
    RCLCPP_ERROR(logger_, "Got ControllerToTeam message with no ControllerReply payload!");
    return false;
  }

  const auto & controller_reply = controller_to_team.controller_reply();

  if (controller_reply.has_status_code() && controller_reply.status_code() != ControllerReply::OK) {
    if (controller_reply.has_reason()) {
      RCLCPP_ERROR(
        logger_, "Game controller sent bad status code (%d) with reason: %s",
        controller_reply.status_code(), controller_reply.reason().c_str());
    } else {
      RCLCPP_ERROR(
        logger_, "Game controller sent bad status code: %d",
        controller_reply.status_code());
    }
    socket_.close();
    return false;
  }

  if (!controller_reply.has_next_token()) {
    RCLCPP_ERROR(logger_, "Controller reply did not include a token!");
  } else {
    next_token_ = controller_reply.next_token();
  }

  return true;
}

bool TeamClient::AttemptToRegister(const std::string & team_name, const TeamColor team_color)
{
  RCLCPP_INFO(logger_, "Registering team...");
  TeamRegistration team_registration_msg;
  team_registration_msg.set_team_name(team_name);
  switch (team_color) {
    case TeamColor::Blue:
      team_registration_msg.set_team(BLUE);
      break;
    case TeamColor::Yellow:
      team_registration_msg.set_team(YELLOW);
      break;
    default:
      break;
  }
  // team_registration_msg.mutable_signature()->set_token(next_token_);

  boost::asio::streambuf boost_streambuf;
  std::ostream std_stream(&boost_streambuf);
  google::protobuf::util::SerializeDelimitedToOstream(team_registration_msg, &std_stream);

  boost::asio::write(socket_, boost_streambuf, boost::asio::transfer_all());

  ControllerToTeam controller_to_team;
  if (!WaitForReply(controller_to_team)) {
    return false;
  }

  if (!controller_to_team.has_controller_reply()) {
    RCLCPP_ERROR(logger_, "Got ControllerToTeam message with no ControllerReply payload!");
    return false;
  }

  const auto & controller_reply = controller_to_team.controller_reply();

  if (controller_reply.has_status_code() && controller_reply.status_code() != ControllerReply::OK) {
    if (controller_reply.has_reason()) {
      RCLCPP_WARN(
        logger_, "Game controller sent bad status code (%d) with reason: %s",
        controller_reply.status_code(), controller_reply.reason().c_str());
    } else {
      RCLCPP_WARN(
        logger_, "Game controller sent bad status code: %d",
        controller_reply.status_code());
    }
    return false;
  }

  return true;
}

bool TeamClient::WaitForReply(ControllerToTeam & reply)
{
  boost::system::error_code error_code;
  rclcpp::WallRate retry_rate(10 /*Hz*/);
  std::size_t bytes_received = 0;
  const auto timeout = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (true) {
    if (std::chrono::steady_clock::now() >= timeout) {
      RCLCPP_ERROR(logger_, "Team client timed out waiting for a reply!");
      return false;
    }
    const auto bytes_available = socket_.available(error_code);
    if (error_code && error_code != boost::asio::error::eof) {
      RCLCPP_ERROR(logger_, "Team client TCP error: %s", error_code.message().c_str());
      return false;
    }
    if (bytes_available == 0) {
      retry_rate.sleep();
      continue;
    }
    bytes_received = boost::asio::read(
      socket_, boost::asio::buffer(
        buffer_), boost::asio::transfer_at_least(bytes_available), error_code);
    if (error_code && error_code != boost::asio::error::eof) {
      RCLCPP_ERROR(logger_, "Team client TCP error: %s", error_code.message().c_str());
      return false;
    }
    break;
  }
  google::protobuf::io::ArrayInputStream array_input_stream(buffer_.data(), bytes_received);
  if (!google::protobuf::util::ParseDelimitedFromZeroCopyStream(
      &reply, &array_input_stream,
      nullptr))
  {
    RCLCPP_ERROR(logger_, "Team client could not parse reply message.");
    return false;
  }

  return true;
}

TeamClient::Result TeamClient::SendRequest(TeamToController & request)
{
  boost::asio::streambuf boost_streambuf;
  std::ostream std_stream(&boost_streambuf);
  google::protobuf::util::SerializeDelimitedToOstream(request, &std_stream);

  boost::asio::write(socket_, boost_streambuf, boost::asio::transfer_all());

  ControllerToTeam controller_to_team;
  if (!WaitForReply(controller_to_team)) {
    return {false, "Client error."};
  }

  if (!controller_to_team.has_controller_reply()) {
    RCLCPP_ERROR(logger_, "Got ControllerToTeam message with no ControllerReply payload!");
    return {false, "Client error."};
  }

  const auto & controller_reply = controller_to_team.controller_reply();

  if (controller_reply.has_next_token()) {
    next_token_ = controller_reply.next_token();
  }

  Result result;
  result.accepted = controller_reply.has_status_code() &&
    (controller_reply.status_code() == ControllerReply::OK);
  if (controller_reply.has_reason()) {
    result.reason = controller_reply.reason();
  }
  return result;
}

}  // namespace ssl_ros_bridge::game_controller_bridge
