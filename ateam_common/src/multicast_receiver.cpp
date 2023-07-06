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

#include "ateam_common/multicast_receiver.hpp"

#include <algorithm>
#include <iostream>
#include <string>

#include <boost/bind/bind.hpp>
#include "ateam_common/get_ip_addresses.hpp"

namespace ateam_common
{

MulticastReceiver::MulticastReceiver(
  std::string multicast_address_string,
  uint16_t multicast_port,
  ReceiveCallback receive_callback)
: receive_callback_(receive_callback)
{
  const auto multicast_address = boost::asio::ip::make_address(multicast_address_string).to_v4();
  const boost::asio::ip::udp::endpoint multicast_endpoint(multicast_address, multicast_port);
  SetupAllSockets(multicast_endpoint, multicast_address);
  io_service_thread_ = std::thread(
    [this]() {
      io_service_.run();
    });
}

MulticastReceiver::~MulticastReceiver()
{
  io_service_.stop();
  if (io_service_thread_.joinable()) {
    io_service_thread_.join();
  }
}

void MulticastReceiver::SendTo(
  const std::string & address, const uint16_t port,
  const char * const data, const size_t length)
{
  if (length >= send_buffer_.size()) {
    std::cout << "WARNING: UDP send data length is larger than buffer" << std::endl;

    return;
  }

  // Copy to buffer
  // Better to send an invalid packet than to overrun the buffer
  // With the if statement above, this should never happen
  memcpy(send_buffer_.data(), data, std::min(send_buffer_.size(), length));

  boost::asio::ip::udp::endpoint endpoint(boost::asio::ip::address::from_string(address), port);

  // TODO which socket is actually best to use?
  sockets_.at(0).async_send_to(
    boost::asio::buffer(send_buffer_, length),
    endpoint,
    boost::bind(
      &MulticastReceiver::HandleUDPSendTo, this,
      boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

void MulticastReceiver::HandleMulticastReceiveFrom(
  boost::asio::ip::udp::socket & socket,
  const boost::system::error_code & error,
  size_t bytes_received)
{
  std::cerr << "HandleMulticastReceiveFrom()\n";
  if (error) {
    std::cerr << "Receive from error: " << error.message() << std::endl;
    return;
  }

  receive_callback_(
    sender_endpoint_.address().to_string(), sender_endpoint_.port(),
    buffer_.data(), bytes_received);

  socket.async_receive_from(
    boost::asio::buffer(buffer_), sender_endpoint_,
    boost::bind(
      &MulticastReceiver::HandleMulticastReceiveFrom, this,
      boost::ref(socket), boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}


void MulticastReceiver::HandleUDPSendTo(const boost::system::error_code & error, size_t)
{
  if (error) {
    std::cerr << "Error sending UDP data: " << error.message() << std::endl;
  }
}

void MulticastReceiver::SetupAllSockets(
  const boost::asio::ip::udp::endpoint & multicast_endpoint,
  const boost::asio::ip::address & multicast_address)
{
  const auto ip_addresses = ateam_common::GetIpV4Addresses();
  for (const auto & address : ip_addresses) {
    SetupSocket(address, multicast_endpoint, multicast_address);
  }
}

void MulticastReceiver::SetupSocket(
  const std::string & interface_address,
  const boost::asio::ip::udp::endpoint & multicast_endpoint,
  const boost::asio::ip::address & multicast_address)
{
  std::cerr << "Listening for multicast packets on interface: " << interface_address << '\n';
  sockets_.emplace_back(io_service_);
  
  sockets_.back().open(multicast_endpoint.protocol());
  sockets_.back().set_option(boost::asio::ip::udp::socket::reuse_address(true));
  boost::asio::ip::udp::endpoint anyaddr_endpoint(multicast_endpoint.protocol(), multicast_endpoint.port());
  sockets_.back().bind(anyaddr_endpoint);
  boost::asio::ip::multicast::join_group(multicast_address.to_v4(), boost::asio::ip::make_address_v4(interface_address));

  sockets_.back().async_receive_from(
    boost::asio::buffer(buffer_), sender_endpoint_,
    boost::bind(
      &MulticastReceiver::HandleMulticastReceiveFrom, this,
      boost::ref(sockets_.back()), boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

} // namespace ateam_common
