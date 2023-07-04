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

#include "types/message_conversions.hpp"

#include <tf2/LinearMath/Quaternion.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace ateam_kenobi::message_conversions
{

ateam_msgs::msg::BallState toMsg(const Ball & obj)
{
  ateam_msgs::msg::BallState ball_state_msg;
  ball_state_msg.pose.position.x = obj.pos.x();
  ball_state_msg.pose.position.y = obj.pos.y();
  ball_state_msg.twist.linear.x = obj.vel.x();
  ball_state_msg.twist.linear.y = obj.vel.y();

  return ball_state_msg;
}

ateam_msgs::msg::RobotState toMsg(const Robot & obj)
{
  ateam_msgs::msg::RobotState robot_state_msg;
  robot_state_msg.pose.position.x = obj.pos.x();
  robot_state_msg.pose.position.y = obj.pos.y();
  robot_state_msg.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), obj.theta));
  robot_state_msg.twist.linear.x = obj.vel.x();
  robot_state_msg.twist.linear.y = obj.vel.y();
  robot_state_msg.twist.angular.z = obj.omega;

  return robot_state_msg;
}

ateam_msgs::msg::World toMsg(const World & obj)
{
  ateam_msgs::msg::World world_msg;

  world_msg.current_time =
    rclcpp::Time(
    std::chrono::duration_cast <
    std::chrono::duration<double>(obj.current_time.time_since_epoch).count());

  // world_msg.field = toMsg(obj.field)
  // world_msg.referee_info = toMsg(obj.referee_info)

  world_msg.balls.push_back(toMsg(obj.ball));

  for (const auto & maybe_robot : obj.our_robots) {
    if (maybe_robot.has_value()) {
      world_msg.our_robots.push_back(toMsg(maybe_robot.value()));
    } else {
      world_msg.our_robots.push_back(ateam_msgs::msg::RobotState());
    }
  }

  for (const auto & maybe_robot : obj.their_robots) {
    if (maybe_robot.has_value()) {
      world_msg.their_robots.push_back(toMsg(maybe_robot.value()));
    } else {
      world_msg.their_robots.push_back(ateam_msgs::msg::RobotState());
    }
  }

  return world_msg;
}

}  // namespace ateam_ai::message_conversions
