

import unittest
import rclpy
import launch
import launch_ros.actions
import launch_testing
import launch_testing_ros
import pytest
import socket
import struct

discovery_address = '224.4.20.69'
discovery_port = 42069

@pytest.mark.rostest
def generate_test_description():
  return launch.LaunchDescription([
    launch_ros.actions.Node(
      package='ateam_radio_bridge',
      executable='radio_bridge_node'
    ),
    launch_testing.actions.ReadyToTest()
  ]), locals()

class TestRadioBridgeNode(unittest.TestCase):
  def setUp(self):
    self.context = rclpy.Context()
    rclpy.init(context=self.context)
    self.node = rclpy.create_node('test_radio_bridge_node', context=self.context, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
    self.message_pump = launch_testing_ros.MessagePump(self.node, context=self.context)
    self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

  def tearDown(self):
    self.message_pump.stop()
    self.node.destroy_node()
    rclpy.shutdown(context=self.context)

  def test_0_discoveryResponse(self):
    self.sock.sendto(struct.pack("HHBBHBB", 0, 0, 101, 1, 2, 0, 0), (discovery_address, discovery_port))
    data, address = self.sock.recvfrom(508)
    print(f"Got data from: {address}")
    