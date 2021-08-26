#ifndef MESSAGE_CONVERSIONS_HPP
#define MESSAGE_CONVERSIONS_HPP

#include <ssl_league_msgs/msg/vision_detection_ball.hpp>
#include <ssl_league_msgs/msg/vision_detection_robot.hpp>
#include <ssl_league_msgs/msg/vision_detection_frame.hpp>
#include <ssl_league_msgs/msg/vision_field_line_segment.hpp>
#include <ssl_league_msgs/msg/vision_field_circular_arc.hpp>
#include <ssl_league_msgs/msg/vision_geometry_field_size.hpp>
#include <ssl_league_msgs/msg/vision_geometry_camera_calibration.hpp>
#include <ssl_league_msgs/msg/vision_geometry_data.hpp>
#include <ssl_league_msgs/msg/vision_wrapper.hpp>

#include <ssl_league_protobufs/ssl_vision_detection.pb.h>
#include <ssl_league_protobufs/ssl_vision_geometry.pb.h>
#include <ssl_league_protobufs/ssl_vision_wrapper.pb.h>

namespace ateam_ssl_vision_bridge::message_conversions
{
ssl_league_msgs::msg::VisionDetectionBall fromProto(const SSL_DetectionBall& proto_msg) {}
ssl_league_msgs::msg::VisionDetectionRobot fromProto(const SSL_DetectionRobot& proto_msg) {}
ssl_league_msgs::msg::VisionDetectionFrame fromProto(const SSL_DetectionFrame& proto_msg) {}

ssl_league_msgs::msg::VisionFieldLineSegment fromProto(const SSL_FieldLineSegment& proto_msg) {}
ssl_league_msgs::msg::VisionFieldCircularArc fromProto(const SSL_FieldCicularArc& proto_msg) {}
ssl_league_msgs::msg::VisionGeometryFieldSize fromProto(const SSL_GeometryFieldSize& proto_msg) {}
ssl_league_msgs::msg::VisionGeometryCameraCalibration fromProto(const SSL_GeometryCameraCalibration& proto_msg) {}
ssl_league_msgs::msg::VisionGeometryData fromProto(const SSL_GeometryData& proto_msg) {}

ssl_league_msgs::msg::VisionWrapper fromProto(const SSL_WrapperPacket& proto_msg) {}

}

#endif // MESSAGE_CONVERSIONS_HPP
