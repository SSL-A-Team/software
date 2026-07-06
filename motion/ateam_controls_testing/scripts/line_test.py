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

from ateam_msgs.msg import RobotMotionCommand

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default


command_msg = RobotMotionCommand()
command_msg.body_control_mode = RobotMotionCommand.BCM_POINT_LINE
command_msg.kick_request = RobotMotionCommand.KR_DISABLE
command_msg.line_global_theta = 0.0
command_msg.line_start_x = -2.0
command_msg.line_start_y = 0.0
command_msg.line_target_x = 0.0
command_msg.line_target_y = 0.0
command_msg.line_dir_x = 0.0
command_msg.line_dir_y = 1.0
command_msg.line_velocity = 0.4
command_msg.line_max_vel_colinear = 10.0
command_msg.line_max_vel_perp = 2.0
command_msg.line_max_vel_angular = 5.0
command_msg.line_max_accel_colinear = 2.0
command_msg.line_max_accel_perp = 2.0
command_msg.line_colinear_start_thresh = 0.1


if __name__ == '__main__':
    argparser = argparse.ArgumentParser(
        description='Test node for sending line maneuvers to the robot'
    )
    argparser.add_argument('robot_id', type=int)
    args = argparser.parse_args()

    rclpy.init()
    node = Node('line_test')

    command_pub = node.create_publisher(
        RobotMotionCommand,
        f'/robot_motion_commands/robot{args.robot_id}',
        qos_profile_system_default,
    )

    rate = node.create_rate(100)
    while rclpy.ok():
        rclpy.spin_once(node)
        command_pub.publish(command_msg)
        rate.sleep()

    node.destroy_node()
    rclpy.shutdown()
