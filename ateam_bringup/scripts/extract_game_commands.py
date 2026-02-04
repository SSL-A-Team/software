#!/usr/bin/python3

# Copyright 2025 A Team
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


from datetime import datetime
import rosbag2_py
from rclpy.serialization import deserialize_message
from ssl_league_msgs.msg import Referee

reader = rosbag2_py.SequentialReader()
storage_options = rosbag2_py.StorageOptions(
    uri='/home/matt/Downloads/GameLog_Bracket1_ATeam_RoboJackets/rosbag2_2025_07_18-18_29_21_Group_ATeam_RoboJackets'
)
converter_options = rosbag2_py.ConverterOptions('', '')
reader.open(storage_options, converter_options)


def get_command_name(command):
    """Return the name of a command."""
    command_names = {
        0: 'HALT',
        1: 'STOP',
        2: 'NORMAL_START',
        3: 'FORCE_START',
        4: 'PREPARE_KICKOFF_YELLOW',
        5: 'PREPARE_KICKOFF_BLUE',
        6: 'PREPARE_PENALTY_YELLOW',
        7: 'PREPARE_PENALTY_BLUE',
        8: 'DIRECT_FREE_YELLOW',
        9: 'DIRECT_FREE_BLUE',
        12: 'TIMEOUT_YELLOW',
        13: 'TIMEOUT_BLUE',
        14: 'GOAL_YELLOW',
        15: 'GOAL_BLUE',
        16: 'BALL_PLACEMENT_YELLOW',
        17: 'BALL_PLACEMENT_BLUE',
    }
    return command_names.get(command, f'UNKNOWN_COMMAND ({command})').ljust(22)


def format_time(t):
    """Format a timestamp in nanoseconds to a human-readable string."""
    dt = datetime.fromtimestamp(t / 1e9)
    return dt.strftime('%Y-%m-%d %H:%M:%S.%f')


prev_cmd = 0
prev_timestamp = 0
while reader.has_next():
    (topic, data, t) = reader.read_next()
    if topic == '/referee_messages':
        msg = deserialize_message(data, Referee)
        if msg.command != prev_cmd:
            elapsed = t - prev_timestamp
            elapsed_sec = elapsed / 1e9
            print(f'{format_time(t)}   {get_command_name(msg.command)}   {elapsed_sec:.3f}s')
            prev_cmd = msg.command
            prev_timestamp = t
