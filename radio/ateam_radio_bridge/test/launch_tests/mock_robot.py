#! /usr/bin/python3
"""Implements robot's radio behavior for testing."""

import socket
import struct
import time
from threading import Thread


class MockRobot:
    def __init__(
        self,
        robot_id=0,
        color="yellow",
        discovery_address="224.4.20.69",
        discovery_port=42069,
    ):
        self._connected = False
        self._last_cmd_packet = None
        self._robot_id = robot_id
        self._color = color
        self._goodbye_requested = False
        self._async_running = False
        self._async_thread = None
        self._bridge_endpoint = ('', 0)
        self._discovery_endpoint = (discovery_address, discovery_port)
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self._socket.settimeout(1.0)

    def isConnected(self) -> bool:
        return self._connected

    def getLastCmdMessage(self) -> bytes:
        return self._last_cmd_packet

    def sendGoodbyeAndShutdown(self) -> None:
        if not self._connected:
            return
        self._goodbye_requested = True
        time.sleep(0.1)
        self.stopAsync()

    def run(self, run_async=False):
        while True:
            if run_async and not self._async_running:
                return
            if not self._connected:
                self._runDiscovery()
            elif self._goodbye_requested:
                self._sendGoodbyePacket()
                self._goodbye_requested = False
            else:
                self._runConnected()

    def startAsync(self):
        self._async_thread = Thread(target=self.run, args=[True])
        self._async_running = True
        self._async_thread.start()

    def stopAsync(self):
        if self._async_thread is None:
            return
        self._async_running = False
        self._async_thread.join()
        self._async_thread = None

    def _runDiscovery(self):
        packet = struct.pack('IBBH BBBBIII',
                             0,  # CRC
                             21,  # CC Hello Req
                             0,  # reserved
                             16,  # Data Length
                             self._robot_id,  # Robot ID
                             1 if self._color == 'blue' else 0,  # Team Color
                             0,  # dirt flags
                             0,  # reserved
                             0,  # coms hash
                             0,  # controls hash
                             0   # firmware hash
                             )
        self._socket.sendto(packet, self._discovery_endpoint)
        try:
            data = self._socket.recv(14)
            if len(data) < 14:
                print(f'Bad packet length: {len(data)}')
                return
            command_code = data[4]
            if command_code != 22:
                print(f'Unexpected command code: {command_code}')
                return
            bridge_port = (data[13] << 8) | data[12]
            bridge_ip = '.'.join([str(x) for x in data[8:12]])
            self._bridge_endpoint = (bridge_ip, bridge_port)
            self._connected = True
            print(f'Connected to radio bridge at {self._bridge_endpoint[0]}:{self._bridge_endpoint[1]}')
        except socket.timeout:
            pass

    def _sendTelemetryPacket(self):
        packet = struct.pack(
            'IBBH BBBBIHHI',
            0,  # CRC
            41,  # CC Telemetry
            0,  # reserved
            16,  # Data Length
            1,  # transmission sequence number
            0,  # control data sequence number
            3,  # control mode (3 = local vel)
            0,  # reserved
            0,  # bit flags
            100,  # battery percent
            0,  # kicker charge percent,
            0,  # control telemetry
        )
        self._socket.sendto(packet, self._bridge_endpoint)
    
    def _sendGoodbyePacket(self):
        packet = struct.pack(
            'IBBH',
            0,  # CRC
            3,  # CC Goodbye
            0,  # reserved
            0,  # Data Length
        )
        self._socket.sendto(packet, self._bridge_endpoint)

    def _runConnected(self):
        try:
            data = self._socket.recv(508)
            command_code = data[4]
            if command_code == 3:
                self._connected = False
                print('Disconnected by request.')
            elif command_code == 61:
                self._last_cmd_packet = data
                self._sendTelemetryPacket()
        except socket.timeout:
            self._connected = False
            print('Disconnected by timeout.')


def main():
    robot = MockRobot()
    robot.run()


if __name__ == "__main__":
    main()
