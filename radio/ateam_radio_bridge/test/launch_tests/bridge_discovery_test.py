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
            struct.pack("IBBHBBBBIII", 653411691, 21, 0, 16, 0, 0, 0, 0, 0, 0, 0), discovery_endpoint
        )
        data, _ = self.sock.recvfrom(16)
        global bridge_endpoint
        self.assertEqual(
            data[4],
            22,
            "Radio bridge sent wrong command code. Expected CC_HELLO_RESP \
                (22).",
        )
        self.assertEqual(
            data[5:7],
            bytearray([0, 6]),
            "Radio bridge sent wrong size for reply data. \
                Expected 6 (size of HelloResponse).",
        )
        # Note: This test does not check full packet contents as they are
        # dynamic and unpredictable from the client
        self.assertNotEqual(
            data[8:12],
            bytearray([0, 0, 0, 0]),
            "Hello response should not hold IP 0.0.0.0",
        )
        self.assertNotEqual(
            data[13:15],
            bytearray([0, 0]),
            "Hello response should not hold \
                port 0",
        )

@launch_testing.post_shutdown_test()
class TestProcessExit(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
