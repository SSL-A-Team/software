#! /usr/bin/env python3

# Copyright 2026 A Team
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import argparse
import time

from ateam_msgs.msg import RobotMotionCommand, VisionStateRobot
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from transforms3d.euler import quat2euler

linear_threshold = 1e-2
angular_threshold = 0.0349

# x, y, theta, hold time
waypoints = [
    (-0.5, -0.5, 0.0, 1.0),
    (-0.5, 0.5, 0.0, 1.0),
    (0.5, 0.5, 0.0, 1.0),
    (0.5, -0.5, 0.0, 1.0),
]

current_index = 0
waypoint_hold_start_time = None
vision_robot_state_msg = None


def vision_callback(msg: VisionStateRobot):
    """Store latest message."""
    global vision_robot_state_msg
    vision_robot_state_msg = msg


def publish_waypoint_command(index: int):
    waypoint = waypoints[index]
    command_msg = RobotMotionCommand()
    command_msg.body_control_mode = RobotMotionCommand.BCM_GLOBAL_POSITION
    command_msg.pose.x = waypoint[0]
    command_msg.pose.y = waypoint[1]
    command_msg.pose.theta = waypoint[2]
    command_msg.kick_request = RobotMotionCommand.KR_DISABLE
    command_msg.limit_acc_linear = 4.0
    command_msg.limit_vel_linear = 2.0
    command_msg.limit_acc_angular = 8.0
    command_msg.limit_vel_angular = 4.0
    command_pub.publish(command_msg)


def is_at_waypoint(index: int):
    waypoint = waypoints[index]
    #  get yaw from quat
    q = vision_robot_state_msg.pose.orientation
    _, _, theta = quat2euler([q.w, q.x, q.y, q.z])
    return (
        vision_robot_state_msg.visible
        and abs(vision_robot_state_msg.pose.position.x - waypoint[0]) < linear_threshold
        and abs(vision_robot_state_msg.pose.position.y - waypoint[1]) < linear_threshold
        and abs(theta - waypoint[2]) < angular_threshold
    )


if __name__ == '__main__':
    argparser = argparse.ArgumentParser(
        description='Test node for sending waypoints to the robot'
    )
    argparser.add_argument('robot_id', type=int)
    argparser.add_argument('--color', '-c', type=str, default='blue')
    args = argparser.parse_args()

    rclpy.init()
    node = Node('waypoints_test')

    command_pub = node.create_publisher(
        RobotMotionCommand,
        f'/robot_motion_commands/robot{args.robot_id}',
        qos_profile_system_default,
    )
    vision_sub = node.create_subscription(
        VisionStateRobot,
        f'/{args.color}_team/robot{args.robot_id}',
        vision_callback,
        qos_profile_system_default,
    )

    while rclpy.ok():
        rclpy.spin_once(node)
        publish_waypoint_command(0)
        if vision_robot_state_msg is None:
            continue
        if is_at_waypoint(0):
            node.get_logger().info('Reached waypoint 0. Starting cycle.')
            break

    while rclpy.ok():
        rclpy.spin_once(node)
        if is_at_waypoint(current_index):
            if waypoint_hold_start_time is None:
                waypoint_hold_start_time = time.time()
            elif (time.time() - waypoint_hold_start_time) > waypoints[current_index][3]:
                current_index = (current_index + 1) % len(waypoints)
                waypoint_hold_start_time = None
                node.get_logger().info(f'Moving to waypoint {current_index}')
        publish_waypoint_command(current_index)

    node.destroy_node()
    rclpy.shutdown()
