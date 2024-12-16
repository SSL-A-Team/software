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

#ifndef VISION_BRIDGE__MESSAGE_CONVERSIONS_HPP_
#define VISION_BRIDGE__MESSAGE_CONVERSIONS_HPP_

#include <ssl_league_protobufs/ssl_vision_detection.pb.h>
#include <ssl_league_protobufs/ssl_vision_geometry.pb.h>
#include <ssl_league_protobufs/ssl_vision_wrapper.pb.h>

#include <ssl_league_msgs/msg/vision_detection_ball.hpp>
#include <ssl_league_msgs/msg/vision_detection_robot.hpp>
#include <ssl_league_msgs/msg/vision_detection_frame.hpp>
#include <ssl_league_msgs/msg/vision_field_line_segment.hpp>
#include <ssl_league_msgs/msg/vision_field_circular_arc.hpp>
#include <ssl_league_msgs/msg/vision_geometry_field_size.hpp>
#include <ssl_league_msgs/msg/vision_geometry_camera_calibration.hpp>
#include <ssl_league_msgs/msg/vision_geometry_data.hpp>
#include <ssl_league_msgs/msg/vision_wrapper.hpp>

namespace ssl_ros_bridge::vision_bridge::message_conversions
{
ssl_league_msgs::msg::VisionDetectionBall fromProto(const SSL_DetectionBall & proto_msg);
ssl_league_msgs::msg::VisionDetectionRobot fromProto(const SSL_DetectionRobot & proto_msg);
ssl_league_msgs::msg::VisionDetectionFrame fromProto(const SSL_DetectionFrame & proto_msg);

ssl_league_msgs::msg::VisionFieldLineSegment fromProto(const SSL_FieldLineSegment & proto_msg);
ssl_league_msgs::msg::VisionFieldCircularArc fromProto(const SSL_FieldCircularArc & proto_msg);
ssl_league_msgs::msg::VisionGeometryFieldSize fromProto(const SSL_GeometryFieldSize & proto_msg);
ssl_league_msgs::msg::VisionGeometryCameraCalibration fromProto(
  const SSL_GeometryCameraCalibration & proto_msg);
ssl_league_msgs::msg::VisionGeometryData fromProto(const SSL_GeometryData & proto_msg);

ssl_league_msgs::msg::VisionWrapper fromProto(const SSL_WrapperPacket & proto_msg);

}  // namespace ssl_ros_bridge::vision_bridge::message_conversions

#endif  // VISION_BRIDGE__MESSAGE_CONVERSIONS_HPP_
