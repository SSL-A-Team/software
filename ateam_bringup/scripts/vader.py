#!/usr/bin/python3

# Copyright 2025 A Team
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

"""A ROS2 service node that kills the ateam_kenobi_node process when called."""

import psutil

import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger


class VaderNode(Node):
    """A ROS2 service node that kills the ateam_kenobi_node process when called."""

    def __init__(self):
        """Initialize the VaderNode."""
        super().__init__('vader')
        self.srv = self.create_service(Trigger, 'strike_him_down', self.callback)

    def callback(self, request, response):
        """Handle service requests to kill the ateam_kenobi_node process."""
        try:
            found_him = False
            for proc in psutil.process_iter():
                if proc.name() == 'ateam_kenobi_node':
                    self.get_logger().info('Killing Kenobi...')
                    proc.kill()
                    response.success = True
                    found_him = True
                    self.get_logger().info('SIGKILL sent.')
            if not found_him:
                self.get_logger().warn('Could not find Kenobi process')
                raise Exception('Could not find Kenobi')
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response


def _main():
    rclpy.init()

    minimal_service = VaderNode()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    _main()
