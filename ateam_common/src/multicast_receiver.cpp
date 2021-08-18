#include "ateam_common/multicast_receiver.hpp"

#include <iostream>

#include <boost/bind.hpp>

namespace ateam_common
{

MulticastReceiver::MulticastReceiver(std::string multicast_address_string,
                                     short multicast_port,
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
  multicast_socket_.async_receive_from(boost::asio::buffer(buffer_), sender_endpoint_, boost::bind(&MulticastReceiver::HandleMulticastReceiveFrom, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

  io_service_thread_ = std::thread([this](){
    io_service_.run();
  });
}

MulticastReceiver::~MulticastReceiver()
{
  io_service_.stop();
  if(io_service_thread_.joinable())
  {
    io_service_thread_.join();
  }
}

void MulticastReceiver::HandleMulticastReceiveFrom(const boost::system::error_code& error, size_t bytes_received)
{
  if(error) {
    std::cerr << "Receive from error: " << error.message() << std::endl;
    return;
  }

  if(!receive_callback_(buffer_.data(), bytes_received)) {
    std::cerr << "Failed to parse message." << std::endl;
    return;
  }

  multicast_socket_.async_receive_from(boost::asio::buffer(buffer_), sender_endpoint_, boost::bind(&MulticastReceiver::HandleMulticastReceiveFrom, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

}