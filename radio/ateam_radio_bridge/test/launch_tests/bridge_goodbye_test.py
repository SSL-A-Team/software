"""Tests the bridge's ability to handle goodbye packets from the robot."""

import time
import unittest

import launch
from launch.actions import TimerAction

import launch_ros.actions

import launch_testing

from mock_robot import MockRobot

import pytest


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

    @classmethod
    def tearDownClass(cls):
        cls.robot.stopAsync()

    def test_feedback(self):
        connect_timeout = time.time() + 1
        while not self.robot.isConnected() and time.time() < connect_timeout:
            time.sleep(0.1)
        self.assertTrue(self.robot.isConnected())

        # Let connection run for a bit
        time.sleep(0.1)

        self.robot.sendGoodbyeAndShutdown()

@launch_testing.post_shutdown_test()
class TestProcessExit(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
