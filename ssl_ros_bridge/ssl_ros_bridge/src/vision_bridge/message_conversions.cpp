// Copyright 2024 A Team
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

#include <tf2/LinearMath/Quaternion.h>

#include <rclcpp/time.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace ssl_ros_bridge::vision_bridge::message_conversions
{

constexpr float mmTom = 1.0e-3f;
constexpr int secToNanosec = 1e9;

ssl_league_msgs::msg::VisionDetectionBall fromProto(const SSL_DetectionBall & proto_msg)
{
  ssl_league_msgs::msg::VisionDetectionBall ros_msg;
  ros_msg.confidence = proto_msg.confidence();
  ros_msg.area = proto_msg.area();  // assuming this is in pixels, not verified
  ros_msg.pos.x = proto_msg.x() * mmTom;
  ros_msg.pos.y = proto_msg.y() * mmTom;
  ros_msg.pos.z = proto_msg.z() * mmTom;
  ros_msg.pixel.x = proto_msg.pixel_x();
  ros_msg.pixel.y = proto_msg.pixel_y();

  return ros_msg;
}
ssl_league_msgs::msg::VisionDetectionRobot fromProto(const SSL_DetectionRobot & proto_msg)
{
  ssl_league_msgs::msg::VisionDetectionRobot ros_msg;
  ros_msg.confidence = proto_msg.confidence();
  ros_msg.robot_id = proto_msg.robot_id();
  ros_msg.pose.position.x = proto_msg.x() * mmTom;
  ros_msg.pose.position.y = proto_msg.y() * mmTom;
  ros_msg.pose.position.z = 0;
  ros_msg.pose.orientation =
    tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), proto_msg.orientation()));
  ros_msg.pixel.x = proto_msg.pixel_x();
  ros_msg.pixel.y = proto_msg.pixel_y();
  ros_msg.height = proto_msg.height();

  return ros_msg;
}
ssl_league_msgs::msg::VisionDetectionFrame fromProto(const SSL_DetectionFrame & proto_msg)
{
  ssl_league_msgs::msg::VisionDetectionFrame ros_msg;
  ros_msg.frame_number = proto_msg.frame_number();
  ros_msg.t_capture = rclcpp::Time(static_cast<int64_t>(proto_msg.t_capture() * secToNanosec));
  ros_msg.t_sent = rclcpp::Time(static_cast<int64_t>(proto_msg.t_sent() * secToNanosec));
  ros_msg.t_capture_camera =
    rclcpp::Time(static_cast<int64_t>(proto_msg.t_capture_camera() * secToNanosec));
  ros_msg.camera_id = proto_msg.camera_id();
  std::transform(
    proto_msg.balls().begin(),
    proto_msg.balls().end(),
    std::back_inserter(ros_msg.balls),
    [](const auto & p) {return fromProto(p);});
  std::transform(
    proto_msg.robots_yellow().begin(),
    proto_msg.robots_yellow().end(),
    std::back_inserter(ros_msg.robots_yellow),
    [](const auto & p) {return fromProto(p);});
  std::transform(
    proto_msg.robots_blue().begin(),
    proto_msg.robots_blue().end(),
    std::back_inserter(ros_msg.robots_blue),
    [](const auto & p) {return fromProto(p);});

  return ros_msg;
}

ssl_league_msgs::msg::VisionFieldLineSegment fromProto(const SSL_FieldLineSegment & proto_msg)
{
  ssl_league_msgs::msg::VisionFieldLineSegment ros_msg;
  ros_msg.name = proto_msg.name();
  ros_msg.p1.x = proto_msg.p1().x() * mmTom;
  ros_msg.p1.y = proto_msg.p1().y() * mmTom;
  ros_msg.p2.x = proto_msg.p2().x() * mmTom;
  ros_msg.p2.y = proto_msg.p2().y() * mmTom;
  ros_msg.thickness = proto_msg.thickness() * mmTom;

  return ros_msg;
}
ssl_league_msgs::msg::VisionFieldCircularArc fromProto(const SSL_FieldCircularArc & proto_msg)
{
  ssl_league_msgs::msg::VisionFieldCircularArc ros_msg;
  ros_msg.name = proto_msg.name();
  ros_msg.center.x = proto_msg.center().x() * mmTom;
  ros_msg.center.y = proto_msg.center().y() * mmTom;
  ros_msg.radius = proto_msg.radius() * mmTom;
  ros_msg.a1 = proto_msg.a1();
  ros_msg.a2 = proto_msg.a2();
  ros_msg.thickness = proto_msg.thickness() * mmTom;

  return ros_msg;
}
ssl_league_msgs::msg::VisionGeometryFieldSize fromProto(const SSL_GeometryFieldSize & proto_msg)
{
  ssl_league_msgs::msg::VisionGeometryFieldSize ros_msg;
  ros_msg.field_length = proto_msg.field_length() * mmTom;
  ros_msg.field_width = proto_msg.field_width() * mmTom;
  ros_msg.goal_width = proto_msg.goal_width() * mmTom;
  ros_msg.goal_depth = proto_msg.goal_depth() * mmTom;
  ros_msg.boundary_width = proto_msg.boundary_width() * mmTom;
  std::transform(
    proto_msg.field_lines().begin(),
    proto_msg.field_lines().end(),
    std::back_inserter(ros_msg.field_lines),
    [](const auto & p) {return fromProto(p);});
  std::transform(
    proto_msg.field_arcs().begin(),
    proto_msg.field_arcs().end(),
    std::back_inserter(ros_msg.field_arcs),
    [](const auto & p) {return fromProto(p);});

  ros_msg.penalty_area_depth = proto_msg.penalty_area_depth() * mmTom;
  ros_msg.penalty_area_width = proto_msg.penalty_area_width() * mmTom;
  ros_msg.center_circle_radius = proto_msg.center_circle_radius() * mmTom;
  ros_msg.line_thickness = proto_msg.line_thickness() * mmTom;
  ros_msg.goal_center_to_penalty_mark = proto_msg.goal_center_to_penalty_mark() * mmTom;
  ros_msg.goal_height = proto_msg.goal_height() * mmTom;
  ros_msg.ball_radius = proto_msg.ball_radius() * mmTom;
  ros_msg.max_robot_radius = proto_msg.max_robot_radius() * mmTom;

  return ros_msg;
}
ssl_league_msgs::msg::VisionGeometryCameraCalibration fromProto(
  const SSL_GeometryCameraCalibration & proto_msg)
{
  ssl_league_msgs::msg::VisionGeometryCameraCalibration ros_msg;
  ros_msg.camera_id = proto_msg.camera_id();
  ros_msg.focal_length = proto_msg.focal_length();
  ros_msg.principal_point.x = proto_msg.principal_point_x();
  ros_msg.principal_point.y = proto_msg.principal_point_y();
  ros_msg.distortion = proto_msg.distortion();
  ros_msg.pose.orientation.x = proto_msg.q0();
  ros_msg.pose.orientation.y = proto_msg.q1();
  ros_msg.pose.orientation.z = proto_msg.q2();
  ros_msg.pose.orientation.w = proto_msg.q3();
  ros_msg.pose.position.x = proto_msg.tx() * mmTom;
  ros_msg.pose.position.y = proto_msg.ty() * mmTom;
  ros_msg.pose.position.z = proto_msg.tz() * mmTom;
  ros_msg.derived_camera_world_t.x = proto_msg.derived_camera_world_tx() * mmTom;
  ros_msg.derived_camera_world_t.y = proto_msg.derived_camera_world_ty() * mmTom;
  ros_msg.derived_camera_world_t.z = proto_msg.derived_camera_world_tz() * mmTom;

  return ros_msg;
}
ssl_league_msgs::msg::VisionGeometryData fromProto(const SSL_GeometryData & proto_msg)
{
  ssl_league_msgs::msg::VisionGeometryData ros_msg;
  ros_msg.field = fromProto(proto_msg.field());
  std::transform(
    proto_msg.calib().begin(),
    proto_msg.calib().end(),
    std::back_inserter(ros_msg.calibration),
    [](const auto & p) {return fromProto(p);});

  return ros_msg;
}

ssl_league_msgs::msg::VisionWrapper fromProto(const SSL_WrapperPacket & proto_msg)
{
  ssl_league_msgs::msg::VisionWrapper ros_msg;
  if (proto_msg.has_detection()) {
    ros_msg.detection.push_back(fromProto(proto_msg.detection()));
  }
  if (proto_msg.has_geometry()) {
    ros_msg.geometry.push_back(fromProto(proto_msg.geometry()));
  }

  return ros_msg;
}

}  // namespace ssl_ros_bridge::vision_bridge::message_conversions
