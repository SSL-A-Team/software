#ifndef MESSAGE_CONVERSIONS_HPP
#define MESSAGE_CONVERSIONS_HPP

#include <ssl_league_msgs/msg/vision.hpp>

#include <ssl_league_protobufs/ssl_vision_detection.pb.h>

namespace ateam_ssl_vision_bridge::message_conversions
{

ssl_league_msgs::msg::Vision fromProto(const SSL_DetectionFrame& proto_msg) {}

}

#endif // MESSAGE_CONVERSIONS_HPP
