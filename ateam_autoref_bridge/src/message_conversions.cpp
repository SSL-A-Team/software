#include "message_conversions.hpp"
#include <rclcpp/time.hpp>

namespace ateam_autoref_bridge::message_conversions
{

ssl_league_msgs::msg::Referee fromProto(const Referee & proto_msg)
{
  ssl_league_msgs::msg::Referee ros_msg;
  ros_msg.timestamp = rclcpp::Time(proto_msg.packet_timestamp() * 1000);
  ros_msg.stage = proto_msg.stage();
  ros_msg.stage_time_left = proto_msg.stage_time_left();
  ros_msg.command = proto_msg.command();
  ros_msg.command_counter = proto_msg.command_counter();
  ros_msg.command_timestamp = rclcpp::Time(proto_msg.command_timestamp() * 1000);
  ros_msg.yellow = fromProto(proto_msg.yellow());
  ros_msg.blue = fromProto(proto_msg.blue());
  ros_msg.designated_position.x = proto_msg.designated_position().x();
  ros_msg.designated_position.y = proto_msg.designated_position().y();
  ros_msg.blue_team_on_positive_half = proto_msg.blue_team_on_positive_half();
  ros_msg.next_command = proto_msg.next_command();
  std::transform(
    proto_msg.game_events().begin(),
    proto_msg.game_events().end(),
    std::back_inserter(ros_msg.game_events),
    [](const auto & p) {return fromProto(p);});
  std::transform(
    proto_msg.game_event_proposals().begin(),
    proto_msg.game_event_proposals().end(),
    std::back_inserter(ros_msg.game_event_proposals),
    [](const auto & p) {return fromProto(p);});
  ros_msg.current_action_time_remaining = proto_msg.current_action_time_remaining();
  return ros_msg;
}

ssl_league_msgs::msg::TeamInfo fromProto(const Referee::TeamInfo & proto_msg)
{
  ssl_league_msgs::msg::TeamInfo ros_msg;
  ros_msg.name = proto_msg.name();
  ros_msg.score = proto_msg.score();
  ros_msg.red_cards = proto_msg.red_cards();
  std::copy(
    proto_msg.yellow_card_times().begin(),
    proto_msg.yellow_card_times().end(), std::back_inserter(ros_msg.yellow_card_times));
  ros_msg.yellow_cards = proto_msg.yellow_cards();
  ros_msg.timeouts = proto_msg.timeouts();
  ros_msg.timeout_time = proto_msg.timeout_time();
  ros_msg.goalkeeper = proto_msg.goalkeeper();
  ros_msg.foul_counter = proto_msg.foul_counter();
  ros_msg.ball_placement_failures = proto_msg.ball_placement_failures();
  ros_msg.can_place_ball = proto_msg.can_place_ball();
  ros_msg.max_allowed_bots = proto_msg.max_allowed_bots();
  ros_msg.bot_substitution_intent = proto_msg.bot_substitution_intent();
  ros_msg.ball_placement_failures_reached = proto_msg.ball_placement_failures_reached();
  return ros_msg;
}

ssl_league_msgs::msg::GameEvent fromProto(const GameEvent & proto_msg)
{
  ssl_league_msgs::msg::GameEvent ros_msg;
  ros_msg.type = proto_msg.type();
  return ros_msg;
}

ssl_league_msgs::msg::GameEventProposalGroup fromProto(const GameEventProposalGroup & proto_msg)
{
  ssl_league_msgs::msg::GameEventProposalGroup ros_msg;
  std::transform(
    proto_msg.game_event().begin(),
    proto_msg.game_event().end(),
    std::back_inserter(ros_msg.game_event),
    [](const auto & p) {return fromProto(p);});
  ros_msg.accepted = proto_msg.accepted();
  return ros_msg;
}

}
