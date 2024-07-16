# Copyright 2021 A Team
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

import launch
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from ateam_bringup.substitutions import PackageLaunchFileSubstitution


def remap_indexed_topics(pattern_pairs):
    return [
        (pattern_from + str(i), pattern_to + str(i))
        for i in range(16)
        for pattern_from, pattern_to in pattern_pairs
    ]


def generate_launch_description():
    return launch.LaunchDescription([
        # Marietta IPs
        # DeclareLaunchArgument("vision_interface_address", default_value="172.16.1.10"),
        # DeclareLaunchArgument("gc_interface_address", default_value="172.16.1.10"),
        # DeclareLaunchArgument("gc_server_address", default_value="172.16.1.52"),
        # DeclareLaunchArgument("radio_interface_address", default_value="172.16.1.10"),

        # Competition IPs
        DeclareLaunchArgument("vision_interface_address", default_value="10.193.15.132"),
        DeclareLaunchArgument("gc_interface_address", default_value="10.193.15.132"),
        DeclareLaunchArgument("gc_server_address", default_value="10.193.12.10"),
        DeclareLaunchArgument("radio_interface_address", default_value="172.16.1.10"),

        DeclareLaunchArgument("team_name", default_value="A-Team"),

        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                PackageLaunchFileSubstitution("ateam_bringup",
                                              "game_controller_nodes.launch.xml")),
            launch_arguments={
                "gc_ip_address": LaunchConfiguration("gc_server_address"),
                "net_interface_address": LaunchConfiguration("gc_interface_address")
            }.items()
        ),

        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                PackageLaunchFileSubstitution("ateam_bringup",
                                              "autonomy.launch.xml")),
            launch_arguments={
                "ssl_vision_interface_address": LaunchConfiguration("vision_interface_address")
            }.items()
        ),

        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                PackageLaunchFileSubstitution("ateam_bringup",
                                              "ui.launch.xml"))
        ),

        Node(
            package="ateam_radio_bridge",
            executable="radio_bridge_node",
            name="radio_bridge",
            parameters=[{
                "net_interface_address": LaunchConfiguration("radio_interface_address"),
                "gc_team_name": LaunchConfiguration("team_name")
            }],
            respawn=True,
            remappings=remap_indexed_topics([
                ("~/robot_motion_commands/robot", "/robot_motion_commands/robot"),
                ("~/robot_feedback/robot", "/robot_feedback/robot")
            ])
        )
    ])
