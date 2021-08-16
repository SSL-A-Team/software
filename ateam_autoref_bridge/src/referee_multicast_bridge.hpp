#ifndef AUTOREF_BRIDGE_HPP
#define AUTOREF_BRIDGE_HPP

#include <ssl_league_msgs/msg/referee.hpp>
#include "ssl_gc_referee_message.pb.h"
#include <boost/asio.hpp>

namespace ateam_autoref_bridge
{
class RefereeMulticastBridge
{
public:
  using RefereePublisher = std::function<void(const ssl_league_msgs::msg::Referee&)>;

  RefereeMulticastBridge(RefereePublisher publisher);

  ~RefereeMulticastBridge();

private:
  RefereePublisher publisher_;
  boost::asio::io_service io_service_;
  boost::asio::ip::udp::socket referee_multicast_socket_;
  boost::asio::ip::udp::endpoint sender_endpoint_;
  std::array<char, 1024> buffer_;
  std::thread io_service_thread_;

  void HandleRefereeMulticastReceiveFrom(const boost::system::error_code& error, size_t bytes_received);
};

}

#endif // AUTOREF_BRIDGE_HPP