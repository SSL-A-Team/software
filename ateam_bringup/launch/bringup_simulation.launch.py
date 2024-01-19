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
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ateam_bringup.substitutions import PackageLaunchFileSubstitution


def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument("start_sim", default_value="True"),
        DeclareLaunchArgument("start_gc", default_value="True"),
        DeclareLaunchArgument("headless_sim", default_value="True"),
        DeclareLaunchArgument("sim_radio_ip", default_value="127.0.0.1"),
        DeclareLaunchArgument("team_name", default_value="A-Team"),

        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                PackageLaunchFileSubstitution("ateam_bringup",
                                              "ssl_grsim.launch.xml")),
            launch_arguments={
                "headless": LaunchConfiguration("headless_sim")
            }.items(),
            condition=IfCondition(LaunchConfiguration("start_sim"))
        ),

        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                PackageLaunchFileSubstitution("ateam_bringup",
                                              "ssl_game_controller.launch.xml")),
            condition=IfCondition(LaunchConfiguration("start_gc"))
        ),

        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                PackageLaunchFileSubstitution("ateam_bringup",
                                              "game_controller_nodes.launch.xml")),
            launch_arguments={
                "net_interface_address": "",
                "gc_ip_address": "127.0.0.1",
                "team_name": LaunchConfiguration("team_name")
            }.items()
        ),

        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                PackageLaunchFileSubstitution("ateam_bringup",
                                              "autonomy.launch.xml")),
            launch_arguments={
                "ssl_vision_interface_address": "",
                "use_world_velocities": "true",
                "ssl_vision_port": "10020",
                "team_name": LaunchConfiguration("team_name")
            }.items()
        ),

        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                PackageLaunchFileSubstitution("ateam_bringup",
                                              "ui.launch.xml"))
        ),

        Node(
            package="ateam_ssl_simulation_radio_bridge",
            executable="ssl_simulation_radio_bridge_node",
            name="radio_bridge",
            parameters=[{
                "ssl_sim_radio_ip": LaunchConfiguration("sim_radio_ip"),
                "gc_team_name": LaunchConfiguration("team_name")
            }]
        )
    ])
