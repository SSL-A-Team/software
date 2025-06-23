"""Tests the bridge's implementation of the discovery handshake."""

import socket
import struct
import unittest
import launch
from launch.actions import TimerAction
import launch_ros.actions
import launch_testing
import pytest

import launch_testing.actions


discovery_address = "224.4.20.70"
discovery_port = 42069
discovery_endpoint = (discovery_address, discovery_port)


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
        cls.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        cls.sock.settimeout(1.0)

    @classmethod
    def tearDownClass(cls):
        cls.sock.close()

    def test_discoveryResponse(self):
        self.sock.sendto(
            struct.pack("IHHBHBB", 653411691, 0, 1, 101, 2, 0, 0), discovery_endpoint
        )
        data, _ = self.sock.recvfrom(52)
        global bridge_endpoint
        self.assertEqual(
            data[8],
            202,
            "Radio bridge sent wrong command code. Expected CC_HELLO_RESP \
                (202).",
        )
        self.assertEqual(
            data[9:11],
            bytearray([0, 6]),
            "Radio bridge sent wrong size for reply data. \
                Expected 6 (size of HelloResponse).",
        )
        # Note: This test does not check full packet contents as they are
        # dynamic and unpredictable from the client
        self.assertNotEqual(
            data[12:16],
            bytearray([0, 0, 0, 0]),
            "Hello response should not hold IP 0.0.0.0",
        )
        self.assertNotEqual(
            data[16:18],
            bytearray([0, 0]),
            "Hello response should not hold \
                port 0",
        )
