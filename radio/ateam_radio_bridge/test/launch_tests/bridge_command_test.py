"""Tests the bridge's ability to pass command messages to robots."""

import socket
import struct
import time
import unittest
import ateam_msgs.msg
import launch
from launch.actions import TimerAction
import launch_ros.actions
import launch_testing

import launch_testing_ros
import pytest
import rclpy
from rclpy.qos import qos_profile_system_default

from mock_robot import MockRobot


discovery_address = "224.4.20.70"
discovery_port = 42069


@pytest.mark.rostest
def generate_test_description():
    return (
        launch.LaunchDescription(
            [
                launch_ros.actions.Node(
                    package="ateam_radio_bridge",
                    executable="radio_bridge_node",
                    parameters=[
                        {"discovery_address": discovery_address},
                        {"default_team_color": "yellow"},
                        {"net_interface_address": ""},
                    ],
                ),
                TimerAction(period=0.1, actions=[launch_testing.actions.ReadyToTest()]),
            ]
        ),
        locals(),
    )


class TestRadioBridgeNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = MockRobot(
            discovery_address=discovery_address, discovery_port=discovery_port
        )
        cls.robot.startAsync()
        cls.context = rclpy.Context()
        rclpy.init(context=cls.context)
        cls.node = rclpy.create_node("test_cmd_pub_node", context=cls.context)
        cls.message_pump = launch_testing_ros.MessagePump(cls.node, context=cls.context)
        cls.message_pump.start()
        cls.cmd_pub = cls.node.create_publisher(
            ateam_msgs.msg.RobotMotionCommand,
            "/radio_bridge/robot_motion_commands/robot0",
            qos_profile_system_default,
        )

    @classmethod
    def tearDownClass(cls):
        cls.robot.stopAsync()
        cls.message_pump.stop()
        cls.node.destroy_node()
        rclpy.shutdown(context=cls.context)

    def test_commands(self):
        connect_timeout = time.time() + 1
        while not self.robot.isConnected() and time.time() < connect_timeout:
            time.sleep(0.1)
        self.assertTrue(self.robot.isConnected())

        cmd_msg = ateam_msgs.msg.RobotMotionCommand()
        cmd_msg.twist.linear.x = 2.0

        timeout = time.time() + 1
        while True:
            time.sleep(0.017)  # 60Hz
            if time.time() > timeout:
                self.fail("timed out waiting for valid command packet")
                break
            self.cmd_pub.publish(cmd_msg)
            last_packet = self.robot.getLastCmdMessage()
            if len(last_packet) != 36:
                continue
            # Extract BasicControl.vel_x_linear
            vel_x_linear = struct.unpack("<f", last_packet[12:16])[0]
            if abs(vel_x_linear - 2.0) < 0.1:
                # Pass the test
                return
