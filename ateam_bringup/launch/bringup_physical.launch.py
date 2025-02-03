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
from ateam_bringup.utils import remap_indexed_topics
import launch
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
                SetLaunchConfiguration('gc_interface_address', '172.17.0.1'),
                SetLaunchConfiguration('gc_server_address', '172.17.0.2'),
            ]),

        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                PackageLaunchFileSubstitution('ateam_bringup',
                                              'league_bridges.launch.xml')),
            launch_arguments={
                'gc_ip_address': LaunchConfiguration('gc_server_address'),
                'gc_net_interface_address': LaunchConfiguration('gc_interface_address'),
                'vision_net_interface_address': LaunchConfiguration('vision_interface_address'),
                'team_name': LaunchConfiguration('team_name')
            }.items()
        ),

        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                PackageLaunchFileSubstitution('ateam_bringup',
                                              'autonomy.launch.xml')),
            launch_arguments={
                'team_name': LaunchConfiguration('team_name')
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
                ('~/robot_feedback/status/robot', '/robot_feedback/status/robot'),
                ('~/robot_feedback/motion/robot', '/robot_feedback/motion/robot')
            ]),
            # prefix=['xterm -bg black -fg white -e gdb -ex run --args']
        ),

        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                PackageLaunchFileSubstitution('ateam_joystick_control',
                                              'joystick_controller.launch.xml')
            )
        )
    ])
