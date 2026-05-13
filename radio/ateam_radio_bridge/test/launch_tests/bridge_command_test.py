"""Tests the bridge's ability to pass command messages to robots."""

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
        cls.vis_pub = cls.node.create_publisher(
            ateam_msgs.msg.VisionStateRobot,
            "/yellow_team/robot0",
            qos_profile_system_default,
        )

    @classmethod
    def tearDownClass(cls):
        cls.robot.stopAsync()
        cls.message_pump.stop()
        cls.node.destroy_node()
        rclpy.shutdown(context=cls.context)

    def test_0_connection(self):
        connect_timeout = time.time() + 1
        while not self.robot.isConnected() and time.time() < connect_timeout:
            time.sleep(0.1)
        self.assertTrue(self.robot.isConnected())

    def test_1_commands(self):
        cmd_msg = ateam_msgs.msg.RobotMotionCommand()
        cmd_msg.body_control_mode = ateam_msgs.msg.RobotMotionCommand.BCM_LOCAL_VELOCITY
        cmd_msg.velocity.x = 2.0

        timeout = time.time() + 1
        while True:
            time.sleep(0.017)  # 60Hz
            if time.time() > timeout:
                self.fail("timed out waiting for valid command packet")
                break
            self.cmd_pub.publish(cmd_msg)
            last_packet = self.robot.getLastCmdMessage()
            if len(last_packet) != 64:
                continue
            # Extract BasicControl.vel_x_linear
            vel_x_linear = struct.unpack("<f", last_packet[36:40])[0]
            if abs(vel_x_linear - 2.0) < 0.1:
                # Pass the test
                return
    
    def test_2_vision_updates(self):
        vis_msg = ateam_msgs.msg.VisionStateRobot()
        vis_msg.pose.position.x = 1.0
        vis_msg.pose.position.y = 2.0
        vis_msg.pose.orientation.x = 0.0
        vis_msg.pose.orientation.y = 0.0
        vis_msg.pose.orientation.z = 0.707
        vis_msg.pose.orientation.w = 0.707
        self.vis_pub.publish(vis_msg)

        timeout = time.time() + 1
        while True:
            time.sleep(0.017)  # 60Hz
            if time.time() > timeout:
                self.fail("timed out waiting for valid vision update packet")
                break
            self.vis_pub.publish(vis_msg)
            last_packet = self.robot.getLastCmdMessage()
            if len(last_packet) != 64:
                continue
            # Extract vision updates
            vision_update_flag = struct.unpack("<B", last_packet[8:9])[0]
            if vision_update_flag & (1 << 6) == 0:
                continue
            vision_x, vision_y, vision_yaw = struct.unpack("<fff", last_packet[12:24])
            if (
                abs(vision_x - 1.00) < 0.01
                and abs(vision_y - 2.00) < 0.01
                and abs(vision_yaw - 1.57) < 0.01
            ):
                # Pass the test
                return

@launch_testing.post_shutdown_test()
class TestProcessExit(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
