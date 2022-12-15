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

#include "message_conversions.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace ateam_vision_filter::message_conversions
{

ateam_msgs::msg::BallState toMsg(const Ball & obj)
{
  ateam_msgs::msg::BallState ball_state_msg;
  ball_state_msg.pose.position.x = obj.position.x();
  ball_state_msg.pose.position.y = obj.position.y();
  ball_state_msg.twist.linear.x = obj.velocity.x();
  ball_state_msg.twist.linear.y = obj.velocity.y();
  ball_state_msg.accel.linear.x = obj.acceleration.x();
  ball_state_msg.accel.linear.y = obj.acceleration.y();

  return ball_state_msg;
}

ateam_msgs::msg::RobotState toMsg(const Robot & obj)
{
  ateam_msgs::msg::RobotState robot_state_msg;
  robot_state_msg.pose.position.x = obj.position.x();
  robot_state_msg.pose.position.y = obj.position.y();
  robot_state_msg.twist.linear.x = obj.velocity.x();
  robot_state_msg.twist.linear.y = obj.velocity.y();
  robot_state_msg.accel.linear.x = obj.acceleration.x();
  robot_state_msg.accel.linear.y = obj.acceleration.y();
  robot_state_msg.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), obj.theta));
  robot_state_msg.twist.angular.z = obj.omega;
  robot_state_msg.accel.angular.z = obj.alpha;

  return robot_state_msg;
}

CameraMeasurement fromMsg(const ssl_league_msgs::msg::VisionWrapper & ros_msg)
{
  return fromMsg(ros_msg.detection);
}

CameraMeasurement fromMsg(const ssl_league_msgs::msg::VisionDetectionFrame & ros_msg)
{
  CameraMeasurement cameraFrame;
  for (const auto & ball_detection : ros_msg.balls) {
    cameraFrame.ball.push_back(fromMsg(ball_detection));
  }

  for (const auto & yellow_robot_detection : ros_msg.robots_yellow) {
    std::size_t robot_id = yellow_robot_detection.robot_id;
    cameraFrame.yellow_robots.at(robot_id).push_back(fromMsg(yellow_robot_detection));
  }

  for (const auto & blue_robot_detection : ros_msg.robots_blue) {
    std::size_t robot_id = blue_robot_detection.robot_id;
    cameraFrame.blue_robots.at(robot_id).push_back(fromMsg(blue_robot_detection));
  }

  return cameraFrame;
}

RobotMeasurement fromMsg(const ssl_league_msgs::msg::VisionDetectionRobot & ros_msg)
{
  tf2::Quaternion tf2_quat;
  RobotMeasurement robotDetection;
  robotDetection.position.x() = ros_msg.pose.position.x;
  robotDetection.position.y() = ros_msg.pose.position.y;
  tf2::fromMsg(ros_msg.pose.orientation, tf2_quat);
  robotDetection.theta = tf2_quat.getAngle();
  return robotDetection;
}

BallMeasurement fromMsg(const ssl_league_msgs::msg::VisionDetectionBall & ros_msg)
{
  BallMeasurement ballDetection;
  ballDetection.position.x() = ros_msg.pos.x;
  ballDetection.position.y() = ros_msg.pos.y;
  return ballDetection;
}

}  // namespace ateam_vision_filter::message_conversions
