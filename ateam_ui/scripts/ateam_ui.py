#!/usr/bin/env python3

# Works when run directly but fails when called from the workspace :/
# Just going to use the xml launch file for now

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rosbridge_server',
            namespace='ui_test',
            executable='rosbridge_websocket.py',
            name='rosbridge'
            )
    ])
