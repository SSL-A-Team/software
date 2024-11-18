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
from launch.actions import IncludeLaunchDescription
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
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                PackageLaunchFileSubstitution('ateam_joystick_control',
                                              'joystick_controller.launch.xml')
            )
        ),
        Node(
            package='ateam_radio_bridge',
            executable='radio_bridge_node',
            name='radio_bridge',
            parameters=[{
                'default_team_color': 'blue'
            }],
            respawn=True,
            remappings=remap_indexed_topics([
                ('~/robot_motion_commands/robot', '/robot_motion_commands/robot'),
                ('~/robot_feedback/status/robot', '/robot_feedback/status/robot'),
                ('~/robot_feedback/motion/robot', '/robot_feedback/motion/robot')
            ])
        )
    ])
