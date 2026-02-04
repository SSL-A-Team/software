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

import cv2

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from ssl_league_msgs.msg import Referee
from ateam_msgs.msg import World
from rclpy.time import Time, ClockType

reader = rosbag2_py.SequentialReader()
storage_options = rosbag2_py.StorageOptions(
    uri='/home/matt/Downloads/GameLog_Group_ATeam_ITAndroids/rosbag2_2025_07_18-12_00_42_Group_ATeam_ITAndroids'
    # uri='/home/matt/Downloads/GameLog_Group_ATeam_RoboFei/rosbag2_2025_07_17-15_00_19_Group_ATeam_RoboFei'
)
converter_options = rosbag2_py.ConverterOptions('', '')
reader.open(storage_options, converter_options)

# msg_defs = reader.get_all_message_definitions()
# for m in msg_defs:
#     if m.topic_type == 'ateam_msgs/msg/World':
#         print(m.encoded_message_definition)
# exit()

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


def from_datetime(dt):
    """Convert a datetime object to a Time object."""
    timestamp = int(dt.timestamp() * 1e9)
    return Time(seconds=timestamp // 1e9, nanoseconds=timestamp % 1e9, clock_type=ClockType.ROS_TIME)


target_time = from_datetime(datetime(2025, 7, 18, 11, 2, 11))
# target_time = from_datetime(datetime(2025, 7, 17, 14, 9, 27))

placement_ref_msg_time = None
placement_ref_msg = None
designated_position = None
world_frame = None

prev_cmd = 0
while reader.has_next():
    (topic, data, t) = reader.read_next()
    if topic == '/referee_messages':
        msg = deserialize_message(data, Referee)
        msg_time = Time.from_msg(msg.timestamp)
        if msg_time >= target_time:
            if msg.command == 16 or msg.command == 17 and len(msg.designated_position) > 0:
                placement_ref_msg_time = t
                placement_ref_msg = msg
                designated_position = msg.designated_position[0]
            if msg.command != prev_cmd and (prev_cmd == 16 or prev_cmd == 17):
                print(f'{format_time(placement_ref_msg_time)}   {get_command_name(placement_ref_msg.command)}')
                print(f'New command: {get_command_name(msg.command)}')
                print(msg.designated_position[0])
                break
        prev_cmd = msg.command
    if topic == '/kenobi_node/world':
        world_frame = deserialize_message(data, World)

width = 9000
height = 6000
field_image = np.zeros((height, width, 3), dtype=np.uint8)

if True:
    cv2.rectangle(field_image, (0, 0), (width, height), (255, 255, 255), thickness=10)
    cv2.line(field_image, (width // 2, 0), (width // 2, height), (255, 255, 255), thickness=10)
    cv2.line(field_image, (0, height // 2), (width, height // 2), (255, 255, 255), thickness=10)
    cv2.circle(field_image, (width // 2, height // 2), 500, (255, 255, 255), thickness=10)
    cv2.rectangle(field_image, (0, (height // 2) - 1000), (1000, (height // 2) + 1000), (255, 255, 255), thickness=10)
    cv2.rectangle(field_image, (width - 1000, (height // 2) - 1000), (width, (height // 2) + 1000), (255, 255, 255), thickness=10)

    if world_frame is not None:
        if len(world_frame.balls) > 0:
            ball = world_frame.balls[0]
            print(f'{ball.pose.position.x}, {ball.pose.position.y}')
            x = int(ball.pose.position.x * 1000) + (width // 2)
            y = int(ball.pose.position.y * 1000) + (height // 2)
            cv2.circle(field_image, (x, y), 50, (0, 164, 255), -1)
            print(f'Ball: {(x,y)}')
        else:
            print('No ball information available in the world frame.')
        for robot in world_frame.our_robots:
            x = int(robot.pose.position.x * 1000) + (width // 2)
            y = int(robot.pose.position.y * 1000) + (height // 2)
            cv2.circle(field_image, (x, y), 90, (255, 0, 0), 10)
        for robot in world_frame.their_robots:
            x = int(robot.pose.position.x * 1000) + (width // 2)
            y = int(robot.pose.position.y * 1000) + (height // 2)
            cv2.circle(field_image, (x, y), 90, (0, 255, 255), 10)

if designated_position is not None:
    center_x = int(designated_position.x * 1000) + (width // 2)
    center_y = int(designated_position.y * 1000) + (height // 2)
    radius = 150
    cv2.circle(field_image, (center_x, center_y), radius, (0, 0, 255), 20)

cv2.imwrite('placement_position.png', field_image)
