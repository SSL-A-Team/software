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
    IncludeLaunchDescription,
)
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return launch.LaunchDescription([
        # IP Overrides
        DeclareLaunchArgument('vision_interface_address', default_value=''),
        DeclareLaunchArgument('radio_interface_address', default_value=''),

        DeclareLaunchArgument('team_name', default_value='A-Team'),

        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                PackageLaunchFileSubstitution('ateam_bringup',
                                              'league_bridges.launch.xml')),
            launch_arguments={
                'vision_net_interface_address': LaunchConfiguration('vision_interface_address'),
                'team_name': LaunchConfiguration('team_name')
            }.items()
        ),

        Node(
            package='ateam_vision_filter',
            executable='ateam_vision_filter_node',
            name='vision_filter',
            parameters=[{
                'gc_team_name': LaunchConfiguration('team_name')
            }],
            respawn=True,
        ),

        Node(
            package='ateam_field_manager',
            executable='ateam_field_manager_node',
            name='field_manager',
            parameters=[{
                'gc_team_name': LaunchConfiguration('team_name')
            }],
            respawn=True,
        ),

        Node(
            package='ateam_game_state',
            executable='game_state_tracker_node',
            name='game_state_tracker',
            parameters=[{
                'gc_team_name': LaunchConfiguration('team_name')
            }],
            respawn=True,
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
                ('~/robot_feedback/basic/robot', '/robot_feedback/basic/robot'),
                ('~/robot_feedback/extended/robot', '/robot_feedback/extended/robot'),
                ('~/robot_feedback/connection/robot', '/robot_feedback/connection/robot'),
                ('~/yellow_team/robot', '/yellow_team/robot'),
                ('~/blue_team/robot', '/blue_team/robot')
            ]),
            # prefix=['xterm -bg black -fg white -e gdb -ex run --args']
        ),

        Node(
            package='ateam_bangbang',
            executable='bangbang_node',
            name='bangbang',
            respawn=True,
        ),

        # IncludeLaunchDescription(
        #     FrontendLaunchDescriptionSource(
        #         PackageLaunchFileSubstitution('ateam_joystick_control',
        #                                       'joystick_controller.launch.xml')
        #     )
        # )
    ])
