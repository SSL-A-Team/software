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

#ifndef ATEAM_COMMON__MULTICAST_RECEIVER_HPP_
#define ATEAM_COMMON__MULTICAST_RECEIVER_HPP_

#include <functional>
#include <string>

#include <boost/asio.hpp>

namespace ateam_common
{
class MulticastReceiver
{
public:
  /**
   * @param uint8_t* Data received in latest packet
   * @param size_t Length of data received
   */
  using ReceiveCallback = std::function<void (uint8_t *, size_t)>;

  MulticastReceiver(
    std::string multicast_ip_address,
    int16_t multicast_port,
    ReceiveCallback receive_callback);

  ~MulticastReceiver();

private:
  ReceiveCallback receive_callback_;
  boost::asio::io_service io_service_;
  boost::asio::ip::udp::socket multicast_socket_;
  boost::asio::ip::udp::endpoint sender_endpoint_;
  std::array<uint8_t, 4096> buffer_;
  std::thread io_service_thread_;

  void HandleMulticastReceiveFrom(const boost::system::error_code & error, size_t bytes_received);
};

}  // namespace ateam_common

#endif  // ATEAM_COMMON__MULTICAST_RECEIVER_HPP_
