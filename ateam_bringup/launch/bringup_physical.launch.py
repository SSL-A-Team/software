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

from ateam_bringup.substitutions import PackageLaunchFileSubstitution
import launch
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def remap_indexed_topics(pattern_pairs):
    return [
        (pattern_from + str(i), pattern_to + str(i))
        for i in range(16)
        for pattern_from, pattern_to in pattern_pairs
    ]


def generate_launch_description():
    return launch.LaunchDescription([
        # IP Overrides
        DeclareLaunchArgument('vision_interface_address', default_value=''),
        DeclareLaunchArgument('radio_interface_address', default_value=''),
        DeclareLaunchArgument('gc_interface_address', default_value=''),
        DeclareLaunchArgument('gc_server_address', default_value=''),

        DeclareLaunchArgument('team_name', default_value='A-Team'),
        DeclareLaunchArgument('use_local_gc', default_value='False'),

        GroupAction(
            condition=IfCondition(LaunchConfiguration('use_local_gc')),
            scoped=False,
            actions=[
                DeclareLaunchArgument('gc_interface_address', default_value='172.17.0.1'),
                DeclareLaunchArgument('gc_server_address', default_value='172.17.0.2'),
            ]),
        GroupAction(
            condition=UnlessCondition(LaunchConfiguration('use_local_gc')),
            scoped=False,
            actions=[
                DeclareLaunchArgument('gc_interface_address', default_value='192.168.1.40'),
                DeclareLaunchArgument('gc_server_address', default_value='192.168.0.114'),
            ]),

        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                PackageLaunchFileSubstitution('ateam_bringup',
                                              'game_controller_nodes.launch.xml')),
            launch_arguments={
                'gc_ip_address': LaunchConfiguration('gc_server_address'),
                'net_interface_address': LaunchConfiguration('gc_interface_address')
            }.items()
        ),

        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                PackageLaunchFileSubstitution('ateam_bringup',
                                              'autonomy.launch.xml')),
            launch_arguments={
                'ssl_vision_interface_address': LaunchConfiguration('vision_interface_address')
            }.items()
        ),

        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                PackageLaunchFileSubstitution('ateam_bringup',
                                              'ui.launch.xml'))
        ),

        Node(
            package='ateam_radio_bridge',
            executable='radio_bridge_node',
            name='radio_bridge',
            parameters=[{
                'net_interface_address': LaunchConfiguration('radio_interface_address'),
                'gc_team_name': LaunchConfiguration('team_name')
            }],
            respawn=True,
            remappings=remap_indexed_topics([
                ('~/robot_motion_commands/robot', '/robot_motion_commands/robot'),
                ('~/robot_feedback/robot', '/robot_feedback/robot')
            ])
        )
    ])
