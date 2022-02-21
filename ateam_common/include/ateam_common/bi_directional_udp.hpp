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

#ifndef ATEAM_COMMON__BI_DIRECTIONAL_UDP_HPP_
#define ATEAM_COMMON__BI_DIRECTIONAL_UDP_HPP_

#include <boost/asio.hpp>

#include <functional>
#include <string>

namespace ateam_common
{
class BiDirectionalUDP
{
public:
  using ReceiveCallback = std::function<void (const char * const data, const size_t length)>;

  BiDirectionalUDP(
    const std::string & udp_ip_address,
    const int16_t udp_port,
    ReceiveCallback receive_callback);

  void send(const char * const data, const size_t length);

  ~BiDirectionalUDP();

private:
  ReceiveCallback receive_callback_;
  boost::asio::io_service io_service_;
  boost::asio::ip::udp::socket udp_socket_;
  boost::asio::ip::udp::endpoint endpoint_;
  std::array<char, 1024> send_buffer_;
  std::array<char, 1024> receive_buffer_;
  std::thread io_service_thread_;

  void HandleUDPSendTo(const boost::system::error_code & error, std::size_t bytes_transferred);
  void HandleUDPReceiveFrom(const boost::system::error_code & error, std::size_t bytes_transferred);
};

}  // namespace ateam_common

#endif  // ATEAM_COMMON__BI_DIRECTIONAL_UDP_HPP_
