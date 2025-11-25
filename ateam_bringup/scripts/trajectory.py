#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_system_default
from pathlib import Path
import json

from ateam_msgs.msg import RobotMotionCommand


class MinimalPublisher(Node):

    def __init__(self, states):
        super().__init__('minimal_publisher')
        # qos_profile = qos_profile_system_default
        # qos_profile = QoSProfile(depth=10)
        # qos_profile.durability = DurabilityPolicy.VOLATILE
        # self.publisher_ = self.create_publisher(RobotMotionCommand, '/robot_motion_commands/robot0', qos_profile)
        self.publisher_ = self.create_publisher(RobotMotionCommand, '/robot_motion_commands/robot0', 10)
        timer_period = 0.01  # seconds
        self.states = states
        self.state_idx = 0
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if self.state_idx < len(self.states):
            current_state = self.states[self.state_idx]
        else:
            current_state = {"x": 0.0, "y": 0.0, "z": 0.0, "x_d": 0.0, "y_d": 0.0, "z_d": 0.0}
        msg = RobotMotionCommand()
        msg.twist.linear.x = current_state["x_d"]
        msg.twist.linear.y = current_state["y_d"]
        msg.twist.angular.z = current_state["z_d"]
        self.publisher_.publish(msg)
        if self.state_idx < len(self.states):
            self.state_idx += 1


def main(args=None):
    states_path = Path(__file__).parent / "states.json"
    with open(states_path, "r") as f:
        states = json.load(f)
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher(states)
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()