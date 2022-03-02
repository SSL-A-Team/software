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
import launch.substitutions
import launch_testing
import launch_testing_ros
import pytest
import os
from ament_index_python.packages import get_package_share_directory


@pytest.mark.rostest
def generate_test_description():
    ateam_ui_src = os.path.join(get_package_share_directory("ateam_ui"), "src")
    ui_process = launch.actions.ExecuteProcess(
        cmd=["neu", "run"],
        cwd=ateam_ui_src,
        output="screen"
    )

    rosbridge_node = launch_ros.actions.Node(
        package="rosbridge_server",
        namespace="ui",
        name="rosbridge",
        executable="rosbridge_websocket.py"
    )

    shutdown_handler = launch.actions.RegisterEventHandler(
        launch.event_handlers.OnProcessExit(
            target_action=ui_process,
            on_exit=[launch.actions.EmitEvent(
                event=launch.events.Shutdown(reason="Window Closed")
            )]
        )
    )

    return launch.LaunchDescription([
        ui_process,
        rosbridge_node,
        shutdown_handler,
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

        # Need time for rosbridge node to set up
        # I want to use the ReadyToTest action but got some weird errors
        time.sleep(5)

        self.message_pump.start()

    def tearDown(self):
        self.message_pump.stop()
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)

    def test_0_subscribers_created(self):

        nodes = self.node.get_node_names()

        # Rosbridge node discovered
        self.assertIn("rosbridge", nodes)

        subscriptions = self.node.get_subscriber_names_and_types_by_node("rosbridge", "/ui")

        # Rosbridge node initialized
        self.assertIsNotNone(subscriptions)

        # Check subscription to /ball
        self.assertIn(("/ball", ['ateam_msgs/msg/BallState']), subscriptions)

        # Check robot subscriptions
        for team in ["yellow", "blue"]:
            for id in range(0, 16):
                topic = (f"/{team}/robot{id}", ['ateam_msgs/msg/RobotState'])
                self.assertIn(topic, subscriptions)

        # Check subscription to /overlay
        self.assertIn(("/overlay", ['ateam_msgs/msg/Overlay']), subscriptions)
