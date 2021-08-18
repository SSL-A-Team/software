#ifndef MULTICAST_RECEIVER_HPP
#define MULTICAST_RECEIVER_HPP

#include <functional>

#include <boost/asio.hpp>

namespace ateam_common
{
class MulticastReceiver
{
public:
  /**
   * @param char* Data received in latest packet
   * @param size_t Length of data received
   * 
   * @return False if failed to parse, True if valid parse
   */
  using ReceiveCallback = std::function<bool(char*, size_t)>;

  MulticastReceiver(std::string multicast_ip_address,
                         short multicast_port,
                         ReceiveCallback receive_callback);

  ~MulticastReceiver();

private:
  ReceiveCallback receive_callback_;
  boost::asio::io_service io_service_;
  boost::asio::ip::udp::socket multicast_socket_;
  boost::asio::ip::udp::endpoint sender_endpoint_;
  std::array<char, 1024> buffer_;
  std::thread io_service_thread_;

  void HandleMulticastReceiveFrom(const boost::system::error_code& error, size_t bytes_received);
};

}

#endif // AUTOREF_BRIDGE_HPP