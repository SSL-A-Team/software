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

import pathlib
import launch
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    AndSubstitution,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
)


def generate_launch_description():
    default_working_dir = pathlib.Path.home() / '.ateam' / 'ssl_game_controller'

    docker_group = GroupAction(
        condition=IfCondition(
            AndSubstitution(
                LaunchConfiguration('gc_use_docker'),
                NotSubstitution(LaunchConfiguration('gc_expose_to_net')),
            )
        ),
        actions=[
            ExecuteProcess(
                cmd=[
                    'docker',
                    'run',
                    '--rm',
                    '-p',
                    '8081:8081',
                    '-p',
                    '10007:10007',
                    '-p',
                    '10008:10008',
                    '-p',
                    '10011:10011',
                    '-v',
                    LaunchConfiguration('gc_working_dir') + '/config:/config',
                    'robocupssl/ssl-game-controller',
                    '-address',
                    ':8081',
                ],
                output='screen',
                respawn=True,
            )
        ],
    )

    docker_exposed_group = GroupAction(
        condition=IfCondition(
            AndSubstitution(
                LaunchConfiguration('gc_use_docker'),
                LaunchConfiguration('gc_expose_to_net'),
            )
        ),
        actions=[
            ExecuteProcess(
                cmd=[
                    'docker',
                    'run',
                    '--rm',
                    '--net',
                    'host',
                    '-v',
                    LaunchConfiguration('gc_working_dir') + '/config:/config',
                    'robocupssl/ssl-game-controller',
                    '-address',
                    ':8081',
                ],
                output='screen',
                respawn=True,
            )
        ],
    )

    native_group = GroupAction(
        condition=UnlessCondition(LaunchConfiguration('gc_use_docker')),
        actions=[
            ExecuteProcess(
                cmd=[
                    LaunchConfiguration('gc_exec_path'),
                    '-address',
                    ':8081',
                ],
                output='screen',
                respawn=True,
            )
        ]
    )

    run_gc_group = GroupAction(
        actions=[
            docker_group,
            docker_exposed_group,
            native_group,
        ]
    )

    create_config_dir = ExecuteProcess(
        cmd=[
            'mkdir',
            '-p',
            PathJoinSubstitution([LaunchConfiguration('gc_working_dir'), 'config']),
        ],
        on_exit=[run_gc_group],
    )

    create_working_dir = ExecuteProcess(
        cmd=['mkdir', '-p', LaunchConfiguration('gc_working_dir')],
        on_exit=[create_config_dir],
    )

    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                'gc_working_dir', default_value=str(default_working_dir)
            ),
            DeclareLaunchArgument(
                'gc_use_docker', default_value='True', choices=['True', 'False']
            ),
            DeclareLaunchArgument(
                'gc_expose_to_net', default_value='False', choices=['True', 'False']
            ),
            DeclareLaunchArgument('gc_exec_path', default_value='ssl-game-controller'),
            create_working_dir,
        ]
    )
