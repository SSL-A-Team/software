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

#ifndef CORE__MULTICAST_RECEIVER_HPP_
#define CORE__MULTICAST_RECEIVER_HPP_

#include <functional>
#include <string>

#include <boost/asio.hpp>

namespace ssl_ros_bridge::core
{
class MulticastReceiver
{
public:
  /**
   * @param sender_address IP address of sender
   * @param sender_port Port number of sender
   * @param data Data received in latest packet
   * @param data_length Length of data received
   */
  using ReceiveCallback =
    std::function<void (const std::string & sender_address, const uint16_t sender_port,
      uint8_t * data, size_t data_length)>;

  MulticastReceiver(
    std::string multicast_ip_address,
    uint16_t multicast_port,
    ReceiveCallback receive_callback,
    std::string interface_address = "");

  ~MulticastReceiver();

  void SendTo(
    const std::string & address, const uint16_t port, const char * const data,
    const size_t length);

private:
  ReceiveCallback receive_callback_;
  boost::asio::io_service io_service_;
  boost::asio::ip::udp::socket multicast_socket_;
  boost::asio::ip::udp::endpoint sender_endpoint_;
  std::array<uint8_t, 4096> send_buffer_;
  std::array<uint8_t, 4096> buffer_;
  std::thread io_service_thread_;

  void HandleMulticastReceiveFrom(const boost::system::error_code & error, size_t bytes_received);

  void HandleUDPSendTo(const boost::system::error_code & error, size_t bytes_sent);

  void JoinMulticastGroupOnAllV4Interfaces(const boost::asio::ip::address & multicast_address);
};

}  // namespace ssl_ros_bridge::core

#endif  // CORE__MULTICAST_RECEIVER_HPP_
