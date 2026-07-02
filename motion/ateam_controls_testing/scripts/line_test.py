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
import math
import time

from ateam_msgs.msg import RobotMotionCommand, VisionStateRobot
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from transforms3d.euler import quat2euler

linear_threshold = 1e-2
angular_threshold = 0.0349

# theta, start_x, start_y, dir_x, dir_y, velocity, hold_time
waypoints = [
    (0.0, -2.0, 0.0, 0.0, 1.0, 0.4, 10.0),
]

line_params = {
    "line_max_vel_perp": 2.0,
    "line_max_vel_angular": 5.0,
    "line_max_accel_colinear": 2.0,
    "line_max_accel_perp": 2.0,
    "line_max_accel_angular": 10.0,
    "line_colinear_start_thresh": 0.1,
}

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
    # command_msg.body_control_mode = RobotMotionCommand.BCM_HEADING_LINE
    command_msg.body_control_mode = RobotMotionCommand.BCM_POINT_LINE
    command_msg.kick_request = RobotMotionCommand.KR_DISABLE
    command_msg.line_global_theta = waypoint[0]
    command_msg.line_start_x = waypoint[1]
    command_msg.line_start_y = waypoint[2]
    command_msg.line_target_x = 0.0
    command_msg.line_target_y = 0.0
    command_msg.line_dir_x = waypoint[3]
    command_msg.line_dir_y = waypoint[4]
    command_msg.line_velocity = waypoint[5]
    command_msg.line_max_vel_colinear = waypoint[5]
    
    for key, val in line_params.items():
        setattr(command_msg, key, val)

    command_pub.publish(command_msg)


def is_at_waypoint(index: int):
    waypoint = waypoints[index]
    theta_target, start_x, start_y, dir_x, dir_y = waypoint[0:5]
    #  get yaw from quat
    q = vision_robot_state_msg.pose.orientation
    _, _, theta = quat2euler([q.w, q.x, q.y, q.z])

    # perpendicular distance from the robot to the line defined by the start
    # point and the (unit) direction vector: magnitude of the 2D cross product
    # of (robot - start) with the direction.
    rel_x = vision_robot_state_msg.pose.position.x - start_x
    rel_y = vision_robot_state_msg.pose.position.y - start_y
    perp_distance = abs(rel_x * dir_y - rel_y * dir_x)

    return False
    # return (
    #     vision_robot_state_msg.visible
    #     and perp_distance < linear_threshold
    #     and abs(theta - theta_target) < angular_threshold
    # )


if __name__ == '__main__':
    argparser = argparse.ArgumentParser(
        description='Test node for sending line maneuvers to the robot'
    )
    argparser.add_argument('robot_id', type=int)
    argparser.add_argument('--color', '-c', type=str, default='blue')
    args = argparser.parse_args()

    rclpy.init()
    node = Node('line_test')

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
