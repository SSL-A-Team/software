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

#include "util/message_conversions.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

namespace ateam_ai::message_conversions
{

ateam_msgs::msg::Sample3d toMsg(const Sample3d & obj)
{
  ateam_msgs::msg::Sample3d sample_3d_msg;
  sample_3d_msg.time.sec = std::floor(obj.time);
  sample_3d_msg.time.nanosec = std::floor((obj.time - std::floor(obj.time)) * 1e9);
  sample_3d_msg.pose.position.x = obj.pose.x();
  sample_3d_msg.pose.position.y = obj.pose.y();
  sample_3d_msg.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), obj.pose.z()));
  sample_3d_msg.twist.linear.x = obj.vel.x();
  sample_3d_msg.twist.linear.y = obj.vel.y();
  sample_3d_msg.twist.angular.z = obj.vel.z();
  sample_3d_msg.accel.linear.x = obj.accel.x();
  sample_3d_msg.accel.linear.y = obj.accel.y();
  sample_3d_msg.accel.angular.z = obj.accel.z();

  return sample_3d_msg;
}

ateam_msgs::msg::Trajectory toMsg(const Trajectory & obj)
{
  ateam_msgs::msg::Trajectory trajectory_msg;
  for (const auto & sample : obj.samples) {
    trajectory_msg.samples.push_back(toMsg(sample));
  }

  return trajectory_msg;
}

ateam_msgs::msg::BehaviorExecutorState toMsg(const BehaviorExecutorState & obj)
{
  ateam_msgs::msg::BehaviorExecutorState behavior_executor_state_msg;

  for (const auto & maybe_trajectory : obj.previous_trajectories) {
    if (maybe_trajectory.has_value()) {
      behavior_executor_state_msg.previous_trajectories.push_back(toMsg(maybe_trajectory.value()));
    } else {
      behavior_executor_state_msg.previous_trajectories.push_back(ateam_msgs::msg::Trajectory());
    }
  }

  return behavior_executor_state_msg;
}

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

  world_msg.current_time.sec = std::floor(obj.current_time);
  world_msg.current_time.nanosec = std::floor(
    (obj.current_time - std::floor(
      obj.current_time)) * 1e9);

  // world_msg.field = toMsg(obj.field)
  // world_msg.referee_info = toMsg(obj.referee_info)

  world_msg.behavior_executor_state = toMsg(obj.behavior_executor_state);

  for (const auto & ball : obj.balls) {
    world_msg.balls.push_back(toMsg(ball));
  }

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

  for (const auto & maybe_robot : obj.plan_from_our_robots) {
    if (maybe_robot.has_value()) {
      world_msg.plan_from_our_robots.push_back(toMsg(maybe_robot.value()));
    } else {
      world_msg.plan_from_our_robots.push_back(ateam_msgs::msg::RobotState());
    }
  }

  return world_msg;
}

}  // namespace ateam_ai::message_conversions
