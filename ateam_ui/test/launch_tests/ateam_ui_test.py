# import time
import unittest
import rclpy
import launch
import launch_ros.actions
import launch.substitutions
import launch_testing
import launch_testing_ros
import pytest
# import sensor_msgs.msg
# import ateam_msgs.msg
# import rcl_interfaces.srv
# import rcl_interfaces.msg


@pytest.mark.rostest
def generate_test_description():
    return launch.LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=["neu", "run"],
            # I want to use ${find-pkg-share ateam_ui} here but can't get it working in python
            cwd="./install/ateam_ui/share/ateam_ui/src",
            output="screen"
        ),
        launch_ros.actions.Node(
            package='rosbridge_server',
            namespace='ui',
            name='rosbridge',
            executable="rosbridge_websocket.py"
        ),
        launch_testing.actions.ReadyToTest()
    ]), locals()


class TestUI(unittest.TestCase):
    def setUp(self):
        self.context = rclpy.Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node('test_ui_node', context=self.context,
                                      allow_undeclared_parameters=True,
                                      automatically_declare_parameters_from_overrides=True)
        self.message_pump = launch_testing_ros.MessagePump(
            self.node, context=self.context)

        # self.pub = self.node.create_publisher(
        #     sensor_msgs.msg.Joy, '/joystick_control_node/joy', 1)

        # self.sub_0 = self.node.create_subscription(
        #     ateam_msgs.msg.RobotMotionCommand, '/motion_commands/robot_0', self.callback_0, 1)
        # self.received_msg_0 = None

        # self.sub_1 = self.node.create_subscription(
        #     ateam_msgs.msg.RobotMotionCommand, '/motion_commands/robot_1', self.callback_1, 1)
        # self.received_msg_1 = None

        # self.set_parameter_client = self.node.create_client(
        #     rcl_interfaces.srv.SetParameters, '/joystick_control_node/set_parameters')
        # self.assertTrue(self.set_parameter_client.wait_for_service(1.0))

        self.message_pump.start()

    def tearDown(self):
        self.message_pump.stop()
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)

    def test_0_subscribers_created(self):
        nodes = self.node.get_node_names_and_namespaces() #rosbridge not showing up in nodes?
        topics = self.node.get_topic_names_and_types()

        raise Exception(nodes)
        # raise Exception(dir(self.context))

        self.assertEqual(topics[0][0], "test")

#     def setRobotId(self, robot_id):
#         set_param_request = rcl_interfaces.srv.SetParameters.Request()
#         set_param_request.parameters = [
#             rcl_interfaces.msg.Parameter(
#                 name="robot_id",
#                 value=rcl_interfaces.msg.ParameterValue(
#                     type=rcl_interfaces.msg.ParameterType.PARAMETER_INTEGER,
#                     integer_value=robot_id))
#         ]
#         return self.set_parameter_client.call(set_param_request)

#     def test_0_defaultRobotIdIsZero(self):
#         joy_msg = sensor_msgs.msg.Joy()
#         joy_msg.axes = [
#             1.0,
#             1.0,
#             0.0,
#             1.0,
#             0.0,
#             0.0,
#             0.0,
#             0.0
#         ]
#         joy_msg.buttons = [0] * 11

#         while self.received_msg_0 is None and self.received_msg_1 is None:
#             self.pub.publish(joy_msg)
#             time.sleep(0.1)

#         self.assertIsNotNone(self.received_msg_0)
#         self.assertIsNone(self.received_msg_1)

#         self.assertAlmostEqual(
#             self.received_msg_0.twist.linear.x, 1.0)
#         self.assertAlmostEqual(
#             self.received_msg_0.twist.linear.y, 1.0)
#         self.assertAlmostEqual(
#             self.received_msg_0.twist.linear.z, 0.0)
#         self.assertAlmostEqual(
#             self.received_msg_0.twist.angular.x, 0.0)
#         self.assertAlmostEqual(
#             self.received_msg_0.twist.angular.y, 0.0)
#         self.assertAlmostEqual(
#             self.received_msg_0.twist.angular.z, 1.0)

#     def test_1_shouldRespondToRobotIdChanges(self):
#         set_param_response = self.setRobotId(1)
#         if not set_param_response.results[0].successful:
#             self.node.get_logger().error('Setting parameter failed with reason: %s' %
#                                          (set_param_response.results[0].reason))
#         self.assertTrue(set_param_response.results[0].successful)

#         joy_msg = sensor_msgs.msg.Joy()
#         joy_msg.axes = [
#             -1.0,
#             -1.0,
#             0.0,
#             -1.0,
#             0.0,
#             0.0,
#             0.0,
#             0.0
#         ]
#         joy_msg.buttons = [0] * 11

#         while self.received_msg_0 is None and self.received_msg_1 is None:
#             self.pub.publish(joy_msg)
#             time.sleep(0.1)

#         self.assertIsNone(self.received_msg_0)
#         self.assertIsNotNone(self.received_msg_1)

#         self.assertAlmostEqual(
#             self.received_msg_1.twist.linear.x, -1.0)
#         self.assertAlmostEqual(
#             self.received_msg_1.twist.linear.y, -1.0)
#         self.assertAlmostEqual(
#             self.received_msg_1.twist.linear.z, 0.0)
#         self.assertAlmostEqual(
#             self.received_msg_1.twist.angular.x, 0.0)
#         self.assertAlmostEqual(
#             self.received_msg_1.twist.angular.y, 0.0)
#         self.assertAlmostEqual(
#             self.received_msg_1.twist.angular.z, -1.0)

#     def test_2_invalidRobotIdsShouldBeRejected(self):
#         self.assertFalse(self.setRobotId(-1).results[0].successful)
#         self.assertFalse(self.setRobotId(16).results[0].successful)

#     def callback_0(self, msg):
#         self.received_msg_0 = ateam_msgs.msg.RobotMotionCommand()
#         self.received_msg_0 = msg

#     def callback_1(self, msg):
#         self.received_msg_1 = ateam_msgs.msg.RobotMotionCommand()
#         self.received_msg_1 = msg
