#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import psutil


class VaderNode(Node):

    def __init__(self):
        super().__init__('vader')
        self.srv = self.create_service(Trigger, 'strike_him_down', self.callback)

    def callback(self, request, response):
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


def main():
    rclpy.init()

    minimal_service = VaderNode()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()