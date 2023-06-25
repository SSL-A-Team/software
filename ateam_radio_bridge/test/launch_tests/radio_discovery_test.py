import unittest
import rclpy
import launch
from launch.actions import TimerAction
import launch_ros.actions
import launch_testing
import launch_testing_ros
import pytest
import socket
import struct
import ateam_msgs.msg
import time

discovery_address = "224.4.20.70"
discovery_port = 42069
discovery_endpoint = (discovery_address, discovery_port)

bridge_endpoint = ("", 0)


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
                        {"default_team_color": "yellow"}
                        ],
                ),
                # Had to add this delay when upgraded to Humble. Without this,
                # the test fails b/c the bridge never seems to receive the
                # discovery message
                TimerAction(period=0.1, actions=[
                    launch_testing.actions.ReadyToTest()]),
            ]
        ),
        locals(),
    )


class TestRadioBridgeNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.Context()
        rclpy.init(context=cls.context)
        cls.node = rclpy.create_node(
            "test_radio_bridge_node",
            context=cls.context,
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        cls.message_pump = launch_testing_ros.MessagePump(
            cls.node, context=cls.context)
        cls.sock = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        cls.sock.settimeout(1.0)
        cls.message_pump.start()

    @classmethod
    def tearDownClass(cls):
        cls.message_pump.stop()
        cls.node.destroy_node()
        cls.sock.close()
        rclpy.shutdown(context=cls.context)

    def setUp(self):
        self.feedback_sub = self.node.create_subscription(
            ateam_msgs.msg.RobotFeedback,
            "/radio_bridge/robot_feedback/robot0",
            self.feedback_callback_0,
            1,
        )
        self.received_feedback_msg_0 = None

    def test_0_discoveryResponse(self):
        self.sock.sendto(
            struct.pack("IHHBHBB", 653411691, 0, 0, 101, 2, 0, 0),
            discovery_endpoint
        )
        data, sender = self.sock.recvfrom(52)
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
            data[16:18], bytearray([0, 0]), "Hello response should not hold \
                port 0"
        )
        server_port = (data[17] << 8) | data[16]
        server_ip = ".".join([str(x) for x in data[12:16]])
        bridge_endpoint = (
            server_ip,
            server_port,
        )  # Save so we can send things back to the bridge

    def test_1_telemetryMessage(self):
        telemetry_msg_data = struct.pack(
            "IHHBHHBBffIffffff",
            1112834395,
            0,
            0,
            102,
            40,
            1,
            2,
            3,
            4.0,
            5.0,
            349525,
            6.0,
            7.0,
            8.0,
            9.0,
            10.0,
            11.0,
        )
        global bridge_endpoint
        self.sock.sendto(telemetry_msg_data, bridge_endpoint)
        while self.received_feedback_msg_0 is None:
            time.sleep(0.1)
        self.assertEqual(self.received_feedback_msg_0.sequence_number, 1)
        self.assertEqual(self.received_feedback_msg_0.robot_revision_major, 2)
        self.assertEqual(self.received_feedback_msg_0.robot_revision_minor, 3)
        self.assertAlmostEqual(self.received_feedback_msg_0.battery_level, 4.0)
        self.assertAlmostEqual(
            self.received_feedback_msg_0.battery_temperature, 5.0)
        self.assertTrue(self.received_feedback_msg_0.power_error)
        self.assertFalse(self.received_feedback_msg_0.tipped_error)
        self.assertTrue(self.received_feedback_msg_0.breakbeam_error)
        self.assertFalse(self.received_feedback_msg_0.breakbeam_ball_detected)
        self.assertTrue(self.received_feedback_msg_0.accelerometer_0_error)
        self.assertFalse(self.received_feedback_msg_0.accelerometer_1_error)
        self.assertTrue(self.received_feedback_msg_0.gyroscope_0_error)
        self.assertFalse(self.received_feedback_msg_0.gyroscope_1_error)
        self.assertTrue(self.received_feedback_msg_0.motor_0_general_error)
        self.assertFalse(self.received_feedback_msg_0.motor_0_hall_error)
        self.assertTrue(self.received_feedback_msg_0.motor_1_general_error)
        self.assertFalse(self.received_feedback_msg_0.motor_1_hall_error)
        self.assertTrue(self.received_feedback_msg_0.motor_2_general_error)
        self.assertFalse(self.received_feedback_msg_0.motor_2_hall_error)
        self.assertTrue(self.received_feedback_msg_0.motor_3_general_error)
        self.assertFalse(self.received_feedback_msg_0.motor_3_hall_error)
        self.assertTrue(self.received_feedback_msg_0.motor_4_general_error)
        self.assertFalse(self.received_feedback_msg_0.motor_4_hall_error)
        self.assertTrue(self.received_feedback_msg_0.chipper_available)
        self.assertFalse(self.received_feedback_msg_0.kicker_available)
        self.assertAlmostEqual(
            self.received_feedback_msg_0.motor_0_temperature, 6.0)
        self.assertAlmostEqual(
            self.received_feedback_msg_0.motor_1_temperature, 7.0)
        self.assertAlmostEqual(
            self.received_feedback_msg_0.motor_2_temperature, 8.0)
        self.assertAlmostEqual(
            self.received_feedback_msg_0.motor_3_temperature, 9.0)
        self.assertAlmostEqual(
            self.received_feedback_msg_0.motor_4_temperature, 10.0)
        self.assertAlmostEqual(
            self.received_feedback_msg_0.kicker_charge_level, 11.0)

    def test_2_commandMessage(self):
        data = self.sock.recv(508)
        expected_data = bytearray(b"\x00") * 36
        expected_data[0:4] = [92, 255, 234, 93]  # CRC
        expected_data[8] = 201  # Command code: CC_CONTROL
        expected_data[9:11] = [0, 24]  # Data length
        expected_data[32] = 0  # Kicker request: KR_ARM
        self.assertEqual(
            len(data),
            len(expected_data),
            "Radio bridge node sent the wrong number of bytes for a command \
                packet",
        )
        for i in range(min(len(data), len(expected_data))):
            if data[i] != expected_data[i]:
                print(f"@ {i}: {data[i]} != {expected_data[i]}")
        self.assertEqual(
            data, expected_data, "Radio bridge node sent an unexpected command\
                  packet"
        )

    def feedback_callback_0(self, msg):
        self.received_feedback_msg_0 = ateam_msgs.msg.RobotFeedback()
        self.received_feedback_msg_0 = msg
