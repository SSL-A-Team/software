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

from ateam_bringup.substitutions import (
    InterfaceFromAddressSubstitution,
    PackageLaunchFileSubstitution
)
import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ateam_bringup.actions import ChangeGameControllerTeamName, ChangeGameControllerConfig


def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument('start_sim', default_value='True'),
        DeclareLaunchArgument('start_gc', default_value='True'),
        DeclareLaunchArgument('start_ui', default_value='True'),
        DeclareLaunchArgument('headless_sim', default_value='True'),
        DeclareLaunchArgument('sim_ip', default_value='127.0.0.1'),
        DeclareLaunchArgument('gc_ip', default_value='172.17.0.2'),
        DeclareLaunchArgument('team_name', default_value='A-Team'),
        DeclareLaunchArgument('no_kenobi', default_value='False'),
        DeclareLaunchArgument('blue_team', default_value='A-Team'),
        DeclareLaunchArgument('yellow_team', default_value='RoboJackets'),

        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                PackageLaunchFileSubstitution('ateam_bringup',
                                              'ssl_erforce_sim.launch.xml')),
            launch_arguments={
                'headless': LaunchConfiguration('headless_sim')
            }.items(),
            condition=IfCondition(LaunchConfiguration('start_sim'))
        ),

        GroupAction(
            actions=[
                IncludeLaunchDescription(
                    FrontendLaunchDescriptionSource(
                        PackageLaunchFileSubstitution('ateam_bringup',
                                                      'ssl_game_controller.launch.xml'))
                ),
                ChangeGameControllerConfig(gc_address='172.17.0.2', configs={
                    'autoContinue': False
                }),
                ChangeGameControllerTeamName(
                    gc_address='172.17.0.2', color='blue', name=LaunchConfiguration('blue_team')),
                ChangeGameControllerTeamName(
                    gc_address='172.17.0.2', color='yellow', name=LaunchConfiguration('yellow_team')),

            ],
            condition=IfCondition(LaunchConfiguration('start_gc'))
        ),

        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                PackageLaunchFileSubstitution('ateam_bringup',
                                              'league_bridges.launch.xml')),
            launch_arguments={
                'gc_net_interface_address': InterfaceFromAddressSubstitution(LaunchConfiguration('gc_ip')),
                'gc_ip_address': LaunchConfiguration('gc_ip'),
                'vision_net_interface_address': InterfaceFromAddressSubstitution(LaunchConfiguration('sim_ip')),
                'vision_port': '10020',
                'team_name': LaunchConfiguration('team_name')
            }.items()
        ),

        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                PackageLaunchFileSubstitution('ateam_bringup',
                                              'ui.launch.xml')),
            condition=IfCondition(LaunchConfiguration('start_ui'))
        ),

        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                PackageLaunchFileSubstitution('ateam_joystick_control',
                                              'joystick_controller.launch.xml')
            )
        ),

        Node(
            package='ateam_ssl_simulation_radio_bridge',
            executable='ssl_simulation_radio_bridge_node',
            name='radio_bridge',
            parameters=[{
                'ssl_sim_radio_ip': LaunchConfiguration('sim_ip'),
                'gc_team_name': LaunchConfiguration('team_name')
            }]
        ),

        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                PackageLaunchFileSubstitution('ateam_bringup',
                                              'state_tracking.launch.xml')
            ),
            launch_arguments={
                'team_name': LaunchConfiguration('team_name'),
            }.items()
        ),

        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                PackageLaunchFileSubstitution(
                    'ateam_bringup', 'kenobi.launch.xml'
                )
            ),
            condition=UnlessCondition(LaunchConfiguration('no_kenobi'))
        ),
    ])
