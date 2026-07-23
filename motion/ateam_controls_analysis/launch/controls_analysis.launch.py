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

r"""
Launch the controls analysis republisher, optionally playing a ROS bag.

Example invocations (live streaming, robot 2)::

  ros2 launch ateam_controls_analysis controls_analysis.launch.py robot_id:=2

Replay a bag through the node (robot 0)::

  ros2 launch ateam_controls_analysis controls_analysis.launch.py \\
      robot_id:=0 bag:=/home/user/game_bag_friendly_greentea2

Then open PlotJuggler, start the 'ROS2 Topic Subscriber', and select the
/controls_analysis/robot{robot_id} topic (or load the bundled layout).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    robot_id = LaunchConfiguration('robot_id')
    reliability = LaunchConfiguration('reliability')
    bag = LaunchConfiguration('bag')
    rate = LaunchConfiguration('rate')
    loop = LaunchConfiguration('loop')

    has_bag = PythonExpression(["'", bag, "' != ''"])
    loop_flag = PythonExpression(
        ["['--loop'] if '", loop, "' == 'true' else []"])

    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value='0'),
        DeclareLaunchArgument(
            'reliability', default_value='reliable',
            description="'reliable' or 'best_effort' telemetry sub QoS"),
        DeclareLaunchArgument(
            'bag', default_value='',
            description='Path to a ROS bag to replay; empty = live stream'),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('loop', default_value='false'),

        Node(
            package='ateam_controls_analysis',
            executable='controls_analysis_node',
            name='controls_analysis_node',
            output='screen',
            parameters=[{
                'robot_id': robot_id,
                'reliability': reliability,
            }],
        ),

        ExecuteProcess(
            condition=IfCondition(has_bag),
            cmd=['ros2', 'bag', 'play', bag, '--rate', rate, loop_flag],
            output='screen',
        ),
    ])
