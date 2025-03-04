"""Tests the bridge's ability to pass feedback packets to ROS."""

import time
import unittest
import ateam_msgs.msg
import launch
from launch.actions import TimerAction
import launch_ros.actions
import launch_testing
from launch_testing_ros.wait_for_topics import WaitForTopics
import pytest

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
        cls.feedback_topic_name = "/radio_bridge/robot_feedback/status/robot0"
        cls.feedback_waiter = WaitForTopics(
            [
                (
                    cls.feedback_topic_name,
                    ateam_msgs.msg.RobotFeedback,
                )
            ],
            timeout=1
        )

    @classmethod
    def tearDownClass(cls):
        cls.robot.stopAsync()
        cls.feedback_waiter.shutdown()

    def test_feedback(self):
        connect_timeout = time.time() + 1
        while not self.robot.isConnected() and time.time() < connect_timeout:
            time.sleep(0.1)
        self.assertTrue(self.robot.isConnected())

        self.assertTrue(self.feedback_waiter.wait())
        self.assertEqual(
            len(self.feedback_waiter.topics_received()),
            1,
            "Did not receive message on feedback topic.",
        )
        at_least_one_connected_message = False
        for message in self.feedback_waiter.received_messages(self.feedback_topic_name):
            if not message.radio_connected:
                continue
            at_least_one_connected_message = True
            # Just checks a few fields to make sure it's a reasonably valid message
            self.assertEqual(message.sequence_number, 1)
            self.assertAlmostEqual(message.battery_level, 24.0)
        self.assertTrue(at_least_one_connected_message)
