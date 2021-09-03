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

#ifndef MESSAGE_CONVERSIONS_HPP_
#define MESSAGE_CONVERSIONS_HPP_

#include <ssl_league_msgs/msg/referee.hpp>
#include <ssl_league_msgs/msg/team_info.hpp>

#include <ssl_league_protobufs/ssl_gc_referee_message.pb.h>
#include <ssl_league_protobufs/ssl_gc_game_event.pb.h>

namespace ateam_autoref_bridge::message_conversions
{

ssl_league_msgs::msg::Referee fromProto(const Referee & proto_msg);
ssl_league_msgs::msg::TeamInfo fromProto(const Referee::TeamInfo & proto_msg);
ssl_league_msgs::msg::GameEvent fromProto(const GameEvent & proto_msg);
ssl_league_msgs::msg::GameEventProposalGroup fromProto(const GameEventProposalGroup & proto_msg);

}

#endif  // MESSAGE_CONVERSIONS_HPP_
