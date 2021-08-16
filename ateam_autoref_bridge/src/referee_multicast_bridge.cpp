#include "referee_multicast_bridge.hpp"
#include "message_conversions.hpp"
#include <boost/bind.hpp>
#include "ssl_gc_referee_message.pb.h"

namespace ateam_autoref_bridge
{

RefereeMulticastBridge::RefereeMulticastBridge(RefereePublisher publisher)
: publisher_(publisher),
  referee_multicast_socket_(io_service_)
{
  const auto multicast_address = boost::asio::ip::make_address("224.5.23.1");
  const short multicast_port = 10003;
  const boost::asio::ip::udp::endpoint multicast_endpoint(multicast_address, multicast_port);
  referee_multicast_socket_.open(multicast_endpoint.protocol());
  referee_multicast_socket_.set_option(boost::asio::ip::udp::socket::reuse_address(true));
  referee_multicast_socket_.bind(multicast_endpoint);
  referee_multicast_socket_.set_option(boost::asio::ip::multicast::join_group(multicast_address));
  referee_multicast_socket_.async_receive_from(boost::asio::buffer(buffer_), sender_endpoint_, boost::bind(&RefereeMulticastBridge::HandleRefereeMulticastReceiveFrom, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

  io_service_thread_ = std::thread([this](){
    io_service_.run();
  });
  std::cout << "Referee multicast bridge ready!\n";
}

RefereeMulticastBridge::~RefereeMulticastBridge()
{
  io_service_.stop();
  if(io_service_thread_.joinable())
  {
    io_service_thread_.join();
  }
}

void RefereeMulticastBridge::HandleRefereeMulticastReceiveFrom(const boost::system::error_code& error, size_t bytes_received)
{
  if(error) {
    std::cerr << "Receive from error: " << error.message() << std::endl;
    return;
  }

  Referee referee_proto;
  if(!referee_proto.ParseFromArray(buffer_.data(), bytes_received)) {
    std::cerr << "Failed to parse message." << std::endl;
    return;
  }
  publisher_(message_conversions::fromProto(referee_proto));

  referee_multicast_socket_.async_receive_from(boost::asio::buffer(buffer_), sender_endpoint_, boost::bind(&RefereeMulticastBridge::HandleRefereeMulticastReceiveFrom, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

}