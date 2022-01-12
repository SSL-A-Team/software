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

#include <boost/bind.hpp>

#include <iostream>
#include <string>

namespace ateam_common
{

MulticastReceiver::MulticastReceiver(
  std::string multicast_address_string,
  int16_t multicast_port,
  ReceiveCallback receive_callback)
: receive_callback_(receive_callback),
  multicast_socket_(io_service_)
{
  const auto multicast_address = boost::asio::ip::make_address(multicast_address_string);
  const boost::asio::ip::udp::endpoint multicast_endpoint(multicast_address, multicast_port);
  multicast_socket_.open(multicast_endpoint.protocol());
  multicast_socket_.set_option(boost::asio::ip::udp::socket::reuse_address(true));
  multicast_socket_.bind(multicast_endpoint);
  multicast_socket_.set_option(boost::asio::ip::multicast::join_group(multicast_address));
  multicast_socket_.async_receive_from(
    boost::asio::buffer(buffer_), sender_endpoint_,
    boost::bind(
      &MulticastReceiver::HandleMulticastReceiveFrom, this,
      boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

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

void MulticastReceiver::HandleMulticastReceiveFrom(
  const boost::system::error_code & error,
  size_t bytes_received)
{
  if (error) {
    std::cerr << "Receive from error: " << error.message() << std::endl;
    return;
  }

  receive_callback_(buffer_.data(), bytes_received);

  multicast_socket_.async_receive_from(
    boost::asio::buffer(buffer_), sender_endpoint_,
    boost::bind(
      &MulticastReceiver::HandleMulticastReceiveFrom, this,
      boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

}  // namespace ateam_common
