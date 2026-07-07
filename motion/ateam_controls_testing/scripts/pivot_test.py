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
from dataclasses import dataclass
import math
import time

from ateam_msgs.msg import RobotMotionCommand, VisionStateRobot
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from transforms3d.euler import quat2euler
import warnings
warnings.filterwarnings("ignore", category=SyntaxWarning, message="invalid escape sequence")
from angles import shortest_angular_distance
warnings.resetwarnings()

angular_threshold = 0.1


@dataclass
class Target():
    theta: float
    orbit_radius: float
    inset_angle: float
    hold_time: float


targets = [
    Target(0.0, 0.1, math.pi / 2, 1.0),
    Target(math.pi/2, 0.1, math.pi / 2, 1.0),
    Target(math.pi, 0.1, math.pi / 2, 1.0),
    Target(-math.pi/2, 0.1, math.pi / 2, 1.0),
]

current_index = 0
waypoint_hold_start_time = None
vision_robot_state_msg = None


def vision_callback(msg: VisionStateRobot):
    """Store latest message."""
    global vision_robot_state_msg
    vision_robot_state_msg = msg


def publish_command(index: int):
    target = targets[index]
    command_msg = RobotMotionCommand()
    command_msg.body_control_mode = RobotMotionCommand.BCM_HEADING_PIVOT
    command_msg.kick_request = RobotMotionCommand.KR_DISABLE
    command_msg.pivot_global_theta = target.theta
    command_msg.pivot_orbit_radius = target.orbit_radius
    command_msg.pivot_inset_angle = target.inset_angle
    command_msg.pivot_max_angular_acc = 8.0
    command_msg.pivot_max_angular_vel = 4.0
    command_pub.publish(command_msg)


def is_at_target(index: int):
    target = targets[index]
    #  get yaw from quat
    q = vision_robot_state_msg.pose.orientation
    _, _, theta = quat2euler([q.w, q.x, q.y, q.z])
    return (
        vision_robot_state_msg.visible
        and abs(shortest_angular_distance(theta, target.theta)) < angular_threshold
    )


if __name__ == '__main__':
    argparser = argparse.ArgumentParser(
        description='Test node for sending pivots to the robot'
    )
    argparser.add_argument('robot_id', type=int)
    argparser.add_argument('--color', '-c', type=str, default='blue')
    args = argparser.parse_args()

    rclpy.init()
    node = Node('pivot_test')

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
        publish_command(0)
        if vision_robot_state_msg is None:
            continue
        if is_at_target(0):
            node.get_logger().info('Reached waypoint 0. Starting cycle.')
            break

    while rclpy.ok():
        rclpy.spin_once(node)
        if is_at_target(current_index):
            if waypoint_hold_start_time is None:
                waypoint_hold_start_time = time.time()
            elif (time.time() - waypoint_hold_start_time) > targets[current_index].hold_time:
                current_index = (current_index + 1) % len(targets)
                waypoint_hold_start_time = None
                node.get_logger().info(f'Moving to waypoint {current_index}')
        publish_command(current_index)

    node.destroy_node()
    rclpy.shutdown()
