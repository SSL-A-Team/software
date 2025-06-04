// Copyright 2025 A Team
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

#include "message_conversions.hpp"

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

namespace ateam_ui_backend_node::message_conversions
{

nlohmann::json fromMsg(
  const ateam_msgs::msg::World & ros_msg) {

    nlohmann::json json;

    json["time"] = fromMsg(ros_msg.current_time);

    json["field"] = fromMsg(ros_msg.field);
    json["referee_info"] = fromMsg(ros_msg.referee_info);

    json["balls"] = nlohmann::json::array();
    for (auto ball : ros_msg.balls) {
      json["balls"].push_back(fromMsg(ball));
    }

    json["our_robots"] = nlohmann::json::array();
    for (auto robot : ros_msg.our_robots) {
      json["our_robots"].push_back(fromMsg(robot));
    }

    json["their_robots"] = nlohmann::json::array();
    for (auto robot : ros_msg.their_robots) {
      json["their_robots"].push_back(fromMsg(robot));
    }

    json["behavior_executor_state"] = fromMsg(ros_msg.behavior_executor_state);

    json["ball_in_play"] = ros_msg.ball_in_play;
    json["double_touch_enforced"] = ros_msg.double_touch_enforced;
    json["double_touch_id"] = ros_msg.double_touch_id;

    return json;
}

nlohmann::json fromMsg(
  const ateam_msgs::msg::RefereeInfo & ros_msg){

  nlohmann::json json;

  json["our_goalie_id"] = ros_msg.our_goalie_id;
  json["their_goalie_id"] = ros_msg.their_goalie_id;
  json["game_stage"] = ros_msg.game_stage;
  json["game_command"] = ros_msg.game_command;
  json["prev_command"] = ros_msg.prev_command;

  json["designated_position"] = fromMsg(ros_msg.designated_position);

  return json;
}

nlohmann::json fromMsg(
  const ateam_msgs::msg::BallState & ros_msg){

  nlohmann::json json;

  json["pose"] = fromMsg(ros_msg.pose);
  json["twist"] = fromMsg(ros_msg.twist);
  json["accel"] = fromMsg(ros_msg.accel);

  json["visible"] = ros_msg.visible;

  return json;
}

nlohmann::json fromMsg(
  const ateam_msgs::msg::RobotState & ros_msg){

  nlohmann::json json;

  json["pose"] = fromMsg(ros_msg.pose);
  json["twist"] = fromMsg(ros_msg.twist);
  json["accel"] = fromMsg(ros_msg.accel);

  json["visible"] = ros_msg.visible;

  return json;
}

nlohmann::json fromMsg(
  const ateam_msgs::msg::RobotFeedback & ros_msg){

  nlohmann::json json;

  json["radio_connected"] = ros_msg.radio_connected;
  json["sequence_number"] = ros_msg.sequence_number;
  json["robot_revision_major"] = ros_msg.robot_revision_major;
  json["robot_revision_minor"] = ros_msg.robot_revision_minor;
  json["battery_level"] = ros_msg.battery_level;
  json["battery_temperature"] = ros_msg.battery_temperature;
  json["power_error"] = ros_msg.power_error;
  json["tipped_error"] = ros_msg.tipped_error;
  json["breakbream_error"] = ros_msg.breakbeam_error;
  json["breakbeam_ball_detected"] = ros_msg.breakbeam_ball_detected;
  json["accelerometer_0_error"] = ros_msg.accelerometer_0_error;
  json["accelerometer_1_error"] = ros_msg.accelerometer_1_error;
  json["gyroscope_0_error"] = ros_msg.gyroscope_0_error;
  json["gyroscope_1_error"] = ros_msg.gyroscope_1_error;
  json["motor_0_general_error"] = ros_msg.motor_0_general_error;
  json["motor_0_hall_error"] = ros_msg.motor_0_hall_error;
  json["motor_1_general_error"] = ros_msg.motor_1_general_error;
  json["motor_1_hall_error"] = ros_msg.motor_1_hall_error;
  json["motor_2_general_error"] = ros_msg.motor_2_general_error;
  json["motor_2_hall_error"] = ros_msg.motor_2_hall_error;
  json["motor_3_general_error"] = ros_msg.motor_3_general_error;
  json["motor_3_hall_error"] = ros_msg.motor_3_hall_error;
  json["motor_4_general_error"] = ros_msg.motor_4_general_error;
  json["motor_4_hall_error"] = ros_msg.motor_4_hall_error;
  json["chipper_available"] = ros_msg.chipper_available;
  json["kicker_available"] = ros_msg.kicker_available;
  json["motor_0_temperature"] = ros_msg.motor_0_temperature;
  json["motor_1_temperature"] = ros_msg.motor_1_temperature;
  json["motor_2_temperature"] = ros_msg.motor_2_temperature;
  json["motor_3_temperature"] = ros_msg.motor_3_temperature;
  json["motor_4_temperature"] = ros_msg.motor_4_temperature;
  json["kicker_charge_level"] = ros_msg.kicker_charge_level;

  return json;
}

nlohmann::json fromMsg(
  const ateam_msgs::msg::BehaviorExecutorState & ros_msg){

  nlohmann::json json;

  json["previous_trajectories"] = nlohmann::json::array();
  for (auto trajectory : ros_msg.previous_trajectories) {
    json["previous_trajectories"].push_back(fromMsg(trajectory));
  }

  return json;
}

nlohmann::json fromMsg(
  const ateam_msgs::msg::Trajectory & ros_msg) {

  nlohmann::json json;

  json["samples"] = nlohmann::json::array();
  for (auto sample : ros_msg.samples) {
    json["sample"].push_back(fromMsg(sample));
  }

  return json;
}

nlohmann::json fromMsg(
  const ateam_msgs::msg::Sample3d & ros_msg) {

  nlohmann::json json;

  json["time"] = fromMsg(ros_msg.time);

  json["pose"] = fromMsg(ros_msg.pose);
  json["twist"] = fromMsg(ros_msg.twist);
  json["accel"] = fromMsg(ros_msg.accel);

  return json;
}

nlohmann::json fromMsg(
  const ateam_msgs::msg::OverlayArray & ros_msg) {

  nlohmann::json json;
  json["overlays"] = nlohmann::json::array();
  for (auto overlay : ros_msg.overlays) {
    json["overlays"].push_back(fromMsg(overlay));
  }

  return json;
}

nlohmann::json fromMsg(
  const ateam_msgs::msg::Overlay & ros_msg) {

  nlohmann::json json;

  json["ns"] = ros_msg.ns;
  json["name"] = ros_msg.name;
  json["visible"] = ros_msg.visible;
  json["type"] = ros_msg.type;
  json["command"] = ros_msg.command;
  json["position"] = fromMsg(ros_msg.position);
  json["scale"] = fromMsg(ros_msg.scale);
  json["stroke_color"] = ros_msg.stroke_color;
  json["fill_color"] = ros_msg.fill_color;
  json["stroke_width"] = ros_msg.stroke_width;
  json["lifetime"] = ros_msg.lifetime;

  json["points"] = nlohmann::json::array();
  for (auto point : ros_msg.points) {
    json["points"].push_back(fromMsg(point));
  }

  json["heatmap_data"] = nlohmann::json::array();
  for (auto data : ros_msg.heatmap_data) {
    json["heatmap_data"].push_back(data);
  }

  json["heatmap_resolution_width"] = ros_msg.heatmap_resolution_width;
  json["heatmap_resolution_height"] = ros_msg.heatmap_resolution_height;

  json["heatmap_alpha"] = nlohmann::json::array();
  for (auto alpha : ros_msg.heatmap_alpha) {
    json["heatmap_alpha"].push_back(alpha);
  }

  json["text"] = ros_msg.text;
  json["depth"] = ros_msg.depth;
  json["start_angle"] = ros_msg.start_angle;
  json["end_angle"] = ros_msg.end_angle;

  return json;
}

nlohmann::json fromMsg(
  const ateam_msgs::msg::FieldInfo & ros_msg){

  nlohmann::json json;

  json["field_length"] = ros_msg.field_length;
  json["field_width"] = ros_msg.field_width;
  json["goal_width"] = ros_msg.goal_width;
  json["goal_depth"] = ros_msg.goal_depth;
  json["boundary_width"] = ros_msg.boundary_width;
  json["defense_area_width"] = ros_msg.defense_area_width;
  json["defense_area_depth"] = ros_msg.defense_area_depth;

  json["center_circle"] = fromMsg(ros_msg.center_circle);
  json["center_circle_radius"] = ros_msg.center_circle_radius;

  json["field_corners"] = fromMsg(ros_msg.field_corners);

  json["ignore_side"] = ros_msg.ignore_side;

  json["ours"] = fromMsg(ros_msg.ours);
  json["theirs"] = fromMsg(ros_msg.theirs);

  return json;
}

nlohmann::json fromMsg(
  const ateam_msgs::msg::FieldSidedInfo & ros_msg) {

  nlohmann::json json;

  json["defense_area_corners"] = fromMsg(ros_msg.defense_area_corners);

  json["goal_corners"] = fromMsg(ros_msg.goal_corners);

  return json;
}

nlohmann::json fromMsg(
  const ssl_league_msgs::msg::Referee & ros_msg){

  nlohmann::json json;

  json["source_identifier"] = nlohmann::json::array();
  for (auto identifier : ros_msg.source_identifier) {
    json["source_identifier"].push_back(identifier);
  }

  json["match_type"] = nlohmann::json::array();
  for (auto type : ros_msg.match_type) {
    json["match_type"].push_back(type);
  }

  json["timestamp"] = fromMsg(ros_msg.timestamp);

  json["stage"] = ros_msg.stage;

  json["stage_time_left"] = nlohmann::json::array();
  for (auto time : ros_msg.stage_time_left) {
    json["stage_time_left"].push_back(time);
  }

  json["command"] = ros_msg.command;
  json["command_counter"] = ros_msg.command_counter;
  json["command_timestamp"] = fromMsg(ros_msg.command_timestamp);

  json["yellow"] = fromMsg(ros_msg.yellow);
  json["blue"] = fromMsg(ros_msg.blue);

  json["designated_position"] = nlohmann::json::array();
  for (auto position : ros_msg.designated_position) {
    json["designated_position"].push_back(fromMsg(position));
  }

  json["blue_team_on_positive_half"] = nlohmann::json::array();
  for (auto blue_team : ros_msg.blue_team_on_positive_half) {
    json["blue_team_on_positive_half"].push_back(blue_team);
  }

  json["next_command"] = nlohmann::json::array();
  for (auto next_command : ros_msg.next_command) {
    json["next_command"].push_back(next_command);
  }

  // Way too much work for right now,
  // will implement later if the ui ever needs game events

  /*
  json["game_events"] = nlohmann::json::array();
  for (auto event : ros_msg.game_events) {
    json["game_events"].push_back(fromMsg(event));
  }

  json["game_event_proposals"] = nlohmann::json::array();
  for (auto proposal : ros_msg.game_event_proposals) {
    json["game_event_proposals"].push_back(fromMsg(proposal));
  }
  */

  json["current_action_time_remaining"] = nlohmann::json::array();
  for (auto action_time : ros_msg.current_action_time_remaining) {
    json["current_action_time_remaining"].push_back(action_time);
  }

  json["status_message"] = nlohmann::json::array();
  for (auto message : ros_msg.status_message) {
    json["status_message"].push_back(message);
  }

  return json;
}

nlohmann::json fromMsg(
  const ssl_league_msgs::msg::TeamInfo & ros_msg){

  nlohmann::json json;

  json["name"] = ros_msg.name;
  json["score"] = ros_msg.score;
  json["red_cards"] = ros_msg.red_cards;

  json["yellow_card_times"] = nlohmann::json::array();
  for (auto time : ros_msg.yellow_card_times) {
    json["yellow_card_times"].push_back(time);
  }

  json["yellow_cards"] = ros_msg.yellow_cards;
  json["timeouts"] = ros_msg.timeouts;
  json["timeout_time"] = ros_msg.timeout_time;
  json["goalkeeper"] = ros_msg.goalkeeper;
  json["foul_counter"] = ros_msg.foul_counter;
  json["ball_placement_failures"] = ros_msg.ball_placement_failures;
  json["can_place_ball"] = ros_msg.can_place_ball;
  json["max_allowed_bots"] = ros_msg.max_allowed_bots;
  json["bot_substitution_intent"] = ros_msg.bot_substitution_intent;
  json["ball_placement_failures_reached"] = ros_msg.ball_placement_failures_reached;
  json["bot_substitution_allowed"] = ros_msg.bot_substitution_allowed;
  json["bot_substitutions_left"] = ros_msg.bot_substitutions_left;
  json["bot_substitution_time_left"] = ros_msg.bot_substitution_time_left;

  return json;
}

nlohmann::json fromMsg(
  const ateam_msgs::msg::PlayInfo & ros_msg) {

  nlohmann::json json;

  json["name"] = ros_msg.name;
  json["description"] = ros_msg.description;

  return json;
}

nlohmann::json fromMsg(
  const ateam_msgs::msg::PlaybookState & ros_msg) {

  nlohmann::json json;

  json["override_name"] = ros_msg.override_name;
  json["running_play_name"] = ros_msg.running_play_name;
  json["running_play_index"] = ros_msg.running_play_index;

  json["names"] = nlohmann::json::array();
  for (auto name : ros_msg.names) {
    json["names"].push_back(name);
  }

  json["enableds"] = nlohmann::json::array();
  for (auto enabled : ros_msg.enableds) {
    json["enableds"].push_back(enabled);
  }

  json["scores"] = nlohmann::json::array();
  for (auto score : ros_msg.scores) {
    json["scores"].push_back(score);
  }

  return json;
}

nlohmann::json fromMsg(
  const ateam_msgs::msg::JoystickControlStatus & ros_msg) {

  nlohmann::json json;

  json["is_active"] = ros_msg.is_active;
  json["active_id"] = ros_msg.active_id;

  return json;
}


// Services:

ateam_msgs::srv::SetDesiredKeeper::Request toSetDesiredKeeperRequest(
  const nlohmann::json json) {

  ateam_msgs::srv::SetDesiredKeeper::Request request;
  request.desired_keeper = json["desired_keeper"];
  
  return request;
}

nlohmann::json fromSrvResponse(
  const ateam_msgs::srv::SetDesiredKeeper::Response & ros_response) {

  nlohmann::json json;

  json["success"] = ros_response.success;
  json["reason"] = ros_response.reason;

  return json;
}

ateam_msgs::srv::SetPlayEnabled::Request toSetPlayEnabledRequest(
  const nlohmann::json json) {

  ateam_msgs::srv::SetPlayEnabled::Request request;
  request.play_name = json["play_name"];
  request.enabled = json["enabled"];
  
  return request;
}

nlohmann::json fromSrvResponse(
  const ateam_msgs::srv::SetPlayEnabled::Response & ros_response) {

  nlohmann::json json;

  json["success"] = ros_response.success;
  json["reason"] = ros_response.reason;

  return json;
}

ateam_msgs::srv::SetOverridePlay::Request toSetOverridePlayRequest(
  const nlohmann::json json) {

  ateam_msgs::srv::SetOverridePlay::Request request;
  request.play_name = json["play_name"];
  
  return request;
}

nlohmann::json fromSrvResponse(
  const ateam_msgs::srv::SetOverridePlay::Response & ros_response) {

  nlohmann::json json;

  json["success"] = ros_response.success;
  json["reason"] = ros_response.reason;

  return json;
}

ateam_msgs::srv::SetIgnoreFieldSide::Request toSetIgnoreFieldSideRequest(
  const nlohmann::json json) {

  ateam_msgs::srv::SetIgnoreFieldSide::Request request;
  request.ignore_side = json["ignore_side"];
  
  return request;
}

nlohmann::json fromSrvResponse(
  const ateam_msgs::srv::SetIgnoreFieldSide::Response & ros_response) {

  nlohmann::json json;

  json["success"] = ros_response.success;
  json["reason"] = ros_response.reason;

  return json;
}

ateam_msgs::srv::SendSimulatorControlPacket::Request toSendSimulatorControlPacketRequest(
  const nlohmann::json json) {

  ateam_msgs::srv::SendSimulatorControlPacket::Request request;
  request.simulator_control = toSimulatorControlMsg(json["simulator_control"]);
  
  return request;
}

ssl_league_msgs::msg::SimulatorControl toSimulatorControlMsg(
  const nlohmann::json json) {

  ssl_league_msgs::msg::SimulatorControl msg;

  for (auto teleport_ball_command : json["teleport_ball"]) {
    msg.teleport_ball.push_back(toTeleportBallCommandMsg(teleport_ball_command));
  }

  for (auto teleport_robot_command : json["teleport_robot"]) {
    msg.teleport_robot.push_back(toTeleportRobotCommandMsg(teleport_robot_command));
  }

  msg.simulation_speed = json["simulation_speed"];

  return msg;
}

ssl_league_msgs::msg::TeleportBallCommand toTeleportBallCommandMsg(
  const nlohmann::json json) {

  ssl_league_msgs::msg::TeleportBallCommand msg;
  msg.pose = toPoseMsg(json["pose"]);
  msg.twist = toTwistMsg(json["twist"]);

  msg.teleport_safely = json["teleport_safely"];
  msg.roll = json["roll"];

  msg.by_force = json["by_force"];

  return msg;
}

ssl_league_msgs::msg::TeleportRobotCommand toTeleportRobotCommandMsg(
  const nlohmann::json json) {

  ssl_league_msgs::msg::TeleportRobotCommand msg;
  msg.id = toRobotIdMsg(json["id"]);

  msg.pose = toPoseMsg(json["pose"]);
  msg.twist = toTwistMsg(json["twist"]);

  msg.present = json["present"];
  msg.by_force = json["by_force"];

  return msg;
}

ssl_league_msgs::msg::RobotId toRobotIdMsg(
  const nlohmann::json json) {

  ssl_league_msgs::msg::RobotId msg;
  for (uint32_t id : json["id"]) {
    msg.id.push_back(id);
  }

  for (auto team : json["team"]) {
    msg.team.push_back(toTeamMsg(team));
  }

  return msg;
}

ssl_league_msgs::msg::Team toTeamMsg(
  const nlohmann::json json) {

  ssl_league_msgs::msg::Team msg;

  msg.color = json["color"];

  return msg;
}

nlohmann::json fromSrvResponse(
  const ateam_msgs::srv::SendSimulatorControlPacket::Response & ros_response) {

  nlohmann::json json;
  
  json["success"] = ros_response.success;
  json["reason"] = ros_response.reason;

  return json;
}


// Standard ROS Messages
nlohmann::json fromMsg(
  const builtin_interfaces::msg::Time & ros_msg){

  nlohmann::json json;

  json["sec"] = ros_msg.sec;
  json["nanosec"] = ros_msg.nanosec;

  return json;
}

nlohmann::json fromMsg(
  const geometry_msgs::msg::Point32 & ros_msg){

  nlohmann::json json;

  json["x"] = ros_msg.x;
  json["y"] = ros_msg.y;
  json["z"] = ros_msg.z;

  return json;
}

nlohmann::json fromMsg(
  const geometry_msgs::msg::Point & ros_msg){

  nlohmann::json json;

  json["x"] = ros_msg.x;
  json["y"] = ros_msg.y;
  json["z"] = ros_msg.z;

  return json;
}

geometry_msgs::msg::Point toPointMsg(
  const nlohmann::json json){

  geometry_msgs::msg::Point msg;

  msg.x = json["x"];
  msg.y = json["y"];
  msg.z = json["z"];

  return msg;
}

nlohmann::json fromMsg(
  const geometry_msgs::msg::Vector3 & ros_msg){

  nlohmann::json json;

  json["x"] = ros_msg.x;
  json["y"] = ros_msg.y;
  json["z"] = ros_msg.z;

  return json;
}

geometry_msgs::msg::Vector3 toVector3Msg(
  const nlohmann::json json){

  geometry_msgs::msg::Vector3 msg;

  msg.x = json["x"];
  msg.y = json["y"];
  msg.z = json["z"];

  return msg;
}

nlohmann::json fromMsg(
  const geometry_msgs::msg::Quaternion & ros_msg){

  nlohmann::json json;

  json["w"] = ros_msg.w;
  json["x"] = ros_msg.x;
  json["y"] = ros_msg.y;
  json["z"] = ros_msg.z;

  return json;
}

geometry_msgs::msg::Quaternion toQuaternionMsg(
  const nlohmann::json json){

  geometry_msgs::msg::Quaternion msg;

  msg.w = json["w"];
  msg.x = json["x"];
  msg.y = json["y"];
  msg.z = json["z"];

  return msg;
}

nlohmann::json fromMsg(
  const geometry_msgs::msg::Pose & ros_msg){

  nlohmann::json json;

  json["position"] = fromMsg(ros_msg.position);
  json["orientation"] = fromMsg(ros_msg.orientation);

  return json;
}

geometry_msgs::msg::Pose toPoseMsg(
  const nlohmann::json json){

  geometry_msgs::msg::Pose msg;

  msg.position = toPointMsg(json["position"]);
  msg.orientation = toQuaternionMsg(json["orientation"]);

  return msg;
}

nlohmann::json fromMsg(
  const geometry_msgs::msg::Twist & ros_msg){

  nlohmann::json json;

  json["linear"] = fromMsg(ros_msg.linear);
  json["angular"] = fromMsg(ros_msg.angular);

  return json;
}

geometry_msgs::msg::Twist toTwistMsg(
  const nlohmann::json json){

  geometry_msgs::msg::Twist msg;
  msg.linear = toVector3Msg(json["linear"]);
  msg.angular = toVector3Msg(json["angular"]);

  return msg;
}

nlohmann::json fromMsg(
  const geometry_msgs::msg::Accel & ros_msg){

  nlohmann::json json;

  json["linear"] = fromMsg(ros_msg.linear);
  json["angular"] = fromMsg(ros_msg.angular);

  return json;
}

nlohmann::json fromMsg(
  const geometry_msgs::msg::Polygon & ros_msg){

  nlohmann::json json;

  json["points"] = nlohmann::json::array();
  for (auto point : ros_msg.points) {
    json["points"].push_back(fromMsg(point));
  }

  return json;
}

}  // namespace ateam_ui_backend_node::message_conversions
