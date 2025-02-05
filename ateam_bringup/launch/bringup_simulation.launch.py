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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument('start_sim', default_value='True'),
        DeclareLaunchArgument('start_gc', default_value='True'),
        DeclareLaunchArgument('start_ui', default_value='True'),
        DeclareLaunchArgument('headless_sim', default_value='True'),
        DeclareLaunchArgument('sim_radio_ip', default_value='127.0.0.1'),
        DeclareLaunchArgument('team_name', default_value='A-Team'),

        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                PackageLaunchFileSubstitution('ateam_bringup',
                                              'ssl_erforce_sim.launch.xml')),
            launch_arguments={
                'headless': LaunchConfiguration('headless_sim')
            }.items(),
            condition=IfCondition(LaunchConfiguration('start_sim'))
        ),

        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                PackageLaunchFileSubstitution('ateam_bringup',
                                              'ssl_game_controller.launch.xml')),
            condition=IfCondition(LaunchConfiguration('start_gc'))
        ),

        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                PackageLaunchFileSubstitution('ateam_bringup',
                                              'league_bridges.launch.xml')),
            launch_arguments={
                'gc_net_interface_address': '172.17.0.1',
                'gc_ip_address': '172.17.0.2',
                'vision_net_interface_address': '127.0.0.1',
                'vision_port': '10020',
                'team_name': LaunchConfiguration('team_name')
            }.items()
        ),

        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                PackageLaunchFileSubstitution('ateam_bringup',
                                              'autonomy.launch.xml')),
            launch_arguments={
                'team_name': LaunchConfiguration('team_name'),
                'use_emulated_ballsense': 'True'
            }.items()
        ),

        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                PackageLaunchFileSubstitution('ateam_bringup',
                                              'ui.launch.xml')),
            condition=IfCondition(LaunchConfiguration('start_ui'))
        ),

        Node(
            package='ateam_ssl_simulation_radio_bridge',
            executable='ssl_simulation_radio_bridge_node',
            name='radio_bridge',
            parameters=[{
                'ssl_sim_radio_ip': LaunchConfiguration('sim_radio_ip'),
                'gc_team_name': LaunchConfiguration('team_name')
            }]
        ),

        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                PackageLaunchFileSubstitution('ateam_joystick_control',
                                              'joystick_controller.launch.xml')
            )
        )
    ])
