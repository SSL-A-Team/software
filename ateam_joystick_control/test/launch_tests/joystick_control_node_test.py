# Copyright 2021 A Team
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

import time
import unittest
import rclpy
import launch
import launch_ros.actions
import launch_testing
import launch_testing_ros
import pytest
import sensor_msgs.msg
import ateam_msgs.msg
import rcl_interfaces.srv
import rcl_interfaces.msg


@pytest.mark.rostest
def generate_test_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='ateam_joystick_control',
            executable='joystick_control_node',
            parameters=[{
                'command_topic_template': '/motion_commands/robot_{}'
            }]
        ),
        launch_testing.actions.ReadyToTest()
    ]), locals()


class TestJoystickControlNode(unittest.TestCase):
    context = None
    node = None
    message_pump = None
    pub = None
    sub_0 = None
    sub_1 = None
    received_msg_0 = None
    received_msg_1 = None
    set_parameter_client = None

    @classmethod
    def setUpClass(cls):
        TestJoystickControlNode.context = rclpy.Context()
        rclpy.init(context=TestJoystickControlNode.context)
        TestJoystickControlNode.node = rclpy.create_node(
            'test_joystick_control_node',
            context=TestJoystickControlNode.context,
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True)
        TestJoystickControlNode.message_pump = launch_testing_ros.MessagePump(
            TestJoystickControlNode.node,
            context=TestJoystickControlNode.context)
        TestJoystickControlNode.pub = TestJoystickControlNode.node. \
            create_publisher(sensor_msgs.msg.Joy, '/joy', 1)

        TestJoystickControlNode.sub_0 = \
            TestJoystickControlNode.node.create_subscription(
                ateam_msgs.msg.RobotMotionCommand,
                '/motion_commands/robot_0',
                TestJoystickControlNode.callback_0,
                1)

        TestJoystickControlNode.sub_1 = \
            TestJoystickControlNode.node.create_subscription(
                ateam_msgs.msg.RobotMotionCommand,
                '/motion_commands/robot_1',
                TestJoystickControlNode.callback_1,
                1)

        TestJoystickControlNode.set_parameter_client = \
            TestJoystickControlNode.node.create_client(
                rcl_interfaces.srv.SetParameters,
                '/joystick_control_node/set_parameters')

        TestJoystickControlNode.message_pump.start()

    @classmethod
    def tearDownClass(cls):
        cls.message_pump.stop()
        cls.node.destroy_node()
        rclpy.shutdown(context=cls.context)

    def setUp(self):
        TestJoystickControlNode.received_msg_0 = None
        TestJoystickControlNode.received_msg_1 = None

    @classmethod
    def callback_0(cls, msg):
        TestJoystickControlNode.received_msg_0 = msg

    @classmethod
    def callback_1(cls, msg):
        TestJoystickControlNode.received_msg_1 = msg

    def setRobotId(self, robot_id):
        self.assertTrue(TestJoystickControlNode.set_parameter_client
                        .wait_for_service(5.0))
        set_param_request = rcl_interfaces.srv.SetParameters.Request()
        set_param_request.parameters = [
            rcl_interfaces.msg.Parameter(
                name="robot_id",
                value=rcl_interfaces.msg.ParameterValue(
                    type=rcl_interfaces.msg.ParameterType.PARAMETER_INTEGER,
                    integer_value=robot_id))
        ]
        return TestJoystickControlNode.set_parameter_client \
            .call(set_param_request)

    def test_0_defaultRobotIdIsZero(self):
        joy_msg = sensor_msgs.msg.Joy()
        joy_msg.axes = [
            1.0,
            1.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            0.0
        ]
        joy_msg.buttons = [0] * 11

        timeout = time.time() + 1
        while TestJoystickControlNode.received_msg_0 is None and \
                TestJoystickControlNode.received_msg_1 is None:
            TestJoystickControlNode.pub.publish(joy_msg)
            time.sleep(0.1)
            if time.time() >= timeout:
                break

        self.assertIsNotNone(TestJoystickControlNode.received_msg_0)
        self.assertIsNone(TestJoystickControlNode.received_msg_1)

        self.assertAlmostEqual(
            TestJoystickControlNode.received_msg_0.twist.linear.x, 1.0)
        self.assertAlmostEqual(
            TestJoystickControlNode.received_msg_0.twist.linear.y, 1.0)
        self.assertAlmostEqual(
            TestJoystickControlNode.received_msg_0.twist.linear.z, 0.0)
        self.assertAlmostEqual(
            TestJoystickControlNode.received_msg_0.twist.angular.x, 0.0)
        self.assertAlmostEqual(
            TestJoystickControlNode.received_msg_0.twist.angular.y, 0.0)
        self.assertAlmostEqual(
            TestJoystickControlNode.received_msg_0.twist.angular.z, 1.0)

    def test_1_shouldSendStopWhenJoystickSwitchesId(self):
        set_param_response = self.setRobotId(1)
        if not set_param_response.results[0].successful:
            TestJoystickControlNode.node.get_logger() \
                .error(f"Setting parameter failed with reason: \
                   {set_param_response.results[0].reason}")
        self.assertTrue(set_param_response.results[0].successful)

        timeout = time.time() + 1
        while TestJoystickControlNode.received_msg_0 is None:
            time.sleep(0.1)
            if time.time() >= timeout:
                break

        self.assertIsNotNone(TestJoystickControlNode.received_msg_0)
        self.assertAlmostEqual(
            TestJoystickControlNode.received_msg_0.twist.linear.x, 0.0)
        self.assertAlmostEqual(
            TestJoystickControlNode.received_msg_0.twist.linear.y, 0.0)
        self.assertAlmostEqual(
            TestJoystickControlNode.received_msg_0.twist.linear.z, 0.0)
        self.assertAlmostEqual(
            TestJoystickControlNode.received_msg_0.twist.angular.x, 0.0)
        self.assertAlmostEqual(
            TestJoystickControlNode.received_msg_0.twist.angular.y, 0.0)
        self.assertAlmostEqual(
            TestJoystickControlNode.received_msg_0.twist.angular.z, 0.0)

    def test_2_afterIdChangeCommandsShouldGoToNewRobot(self):
        joy_msg = sensor_msgs.msg.Joy()
        joy_msg.axes = [
            -1.0,
            -1.0,
            0.0,
            -1.0,
            0.0,
            0.0,
            0.0,
            0.0
        ]
        joy_msg.buttons = [0] * 11

        timeout = time.time() + 1
        while TestJoystickControlNode.received_msg_1 is None:
            TestJoystickControlNode.pub.publish(joy_msg)
            time.sleep(0.1)
            if time.time() >= timeout:
                break

        self.assertIsNotNone(TestJoystickControlNode.received_msg_1)
        self.assertAlmostEqual(
            TestJoystickControlNode.received_msg_1.twist.linear.x, -1.0)
        self.assertAlmostEqual(
            TestJoystickControlNode.received_msg_1.twist.linear.y, -1.0)
        self.assertAlmostEqual(
            TestJoystickControlNode.received_msg_1.twist.linear.z, 0.0)
        self.assertAlmostEqual(
            TestJoystickControlNode.received_msg_1.twist.angular.x, 0.0)
        self.assertAlmostEqual(
            TestJoystickControlNode.received_msg_1.twist.angular.y, 0.0)
        self.assertAlmostEqual(
            TestJoystickControlNode.received_msg_1.twist.angular.z, -1.0)

    def test_3_invalidRobotIdsShouldBeRejected(self):
        self.assertFalse(self.setRobotId(-2).results[0].successful)
        self.assertFalse(self.setRobotId(16).results[0].successful)
