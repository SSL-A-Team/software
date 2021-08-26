#ifndef MESSAGE_CONVERSIONS_HPP
#define MESSAGE_CONVERSIONS_HPP

#include <ssl_league_msgs/msg/referee.hpp>
#include <ssl_league_msgs/msg/team_info.hpp>

#include <ssl_league_protobufs/ssl_gc_referee_message.pb.h>
#include <ssl_league_protobufs/ssl_gc_game_event.pb.h>

namespace ateam_autoref_bridge::message_conversions
{

ssl_league_msgs::msg::Referee fromProto(const Referee& proto_msg);
ssl_league_msgs::msg::TeamInfo fromProto(const Referee::TeamInfo& proto_msg);
ssl_league_msgs::msg::GameEvent fromProto(const GameEvent& proto_msg);
ssl_league_msgs::msg::GameEventProposalGroup fromProto(const GameEventProposalGroup& proto_msg);

}

#endif // MESSAGE_CONVERSIONS_HPP
