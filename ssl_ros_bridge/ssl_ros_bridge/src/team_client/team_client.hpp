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

#ifndef TEAM_CLIENT_HPP_
#define TEAM_CLIENT_HPP_

#include <ssl_league_protobufs/ssl_gc_rcon_team.pb.h>
#include <array>
#include <atomic>
#include <mutex>
#include <string>
#include <vector>
#include <boost/asio.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ssl_ros_bridge::game_controller_bridge
{

class TeamClient
{
public:
  struct Result
  {
    bool accepted;
    std::string reason;
  };

  struct PingResult
  {
    Result request_result;
    std::chrono::duration<double, std::milli> ping;
  };

  enum class TeamColor
  {
    Auto,
    Blue,
    Yellow
  };

  struct ConnectionParameters
  {
    boost::asio::ip::address address;
    uint16_t port;
    std::string team_name;
    TeamColor team_color;
  };

  enum class AdvantageChoiceOption
  {
    Stop,
    Continue
  };

  explicit TeamClient(rclcpp::Logger logger);

  bool Connect(const ConnectionParameters & parameters);

  void Disconnect();

  bool IsConnected() const
  {
    return connected_;
  }

  Result SetDesiredKeeper(const int keeper_id);

  Result RequestBotSubstitution();

  Result SetAdvantageChoice(const AdvantageChoiceOption & choice);

  PingResult Ping();

private:
  rclcpp::Logger logger_;
  bool connected_{false};
  std::string next_token_;
  std::array<char, 1024> buffer_;
  std::size_t buffer_index_{0};
  boost::asio::io_service io_service_;
  boost::asio::ip::tcp::socket socket_;

  bool AttemptToConnectSocket(const boost::asio::ip::address & address, const uint16_t port);

  bool AttemptToRegister(const std::string & team_name, const TeamColor team_color);

  bool WaitForReply(ControllerToTeam & reply);

  Result SendRequest(TeamToController & request);
};

}  // namespace ssl_ros_bridge::game_controller_bridge
#endif  // TEAM_CLIENT_HPP_
