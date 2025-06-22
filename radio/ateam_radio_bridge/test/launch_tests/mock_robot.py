#! /usr/bin/python3
"""Implements robot's radio behavior for testing."""

import socket
import struct
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

    def run(self, run_async=False):
        while True:
            if run_async and not self._async_running:
                return
            if not self._connected:
                self._runDiscovery()
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
        packet = struct.pack('IHHBHBB',
                             0,  # CRC
                             0,  # Version Major
                             1,  # Version Minor
                             101,  # CC Hello Req
                             2,  # Data Length
                             self._robot_id,  # Robot ID
                             1 if self._color == 'blue' else 0,  # Team Color
                             )
        self._socket.sendto(packet, self._discovery_endpoint)
        try:
            data = self._socket.recv(18)
            if len(data) < 18:
                print(f'Bad packet length: {len(data)}')
                return
            command_code = data[8]
            if command_code != 202:
                print(f'Unexpected command code: {command_code}')
                return
            bridge_port = (data[17] << 8) | data[16]
            bridge_ip = '.'.join([str(x) for x in data[12:16]])
            self._bridge_endpoint = (bridge_ip, bridge_port)
            self._connected = True
            print(f'Connected to radio bridge at {self._bridge_endpoint[0]}:{self._bridge_endpoint[1]}')
        except socket.timeout:
            pass

    def _sendTelemetryPacket(self):
        packet = struct.pack(
            'IHHBHHBBIHH',
            0,  # CRC
            0,  # Version Major
            1,  # Version Minor
            102,  # CC Telemetry
            12,  # Data Length
            1,  # Sequence Number
            0,  # Robot Revision Major
            0,  # Robot Revision Minor
            0,  # Bit Flags
            100,  # Battery Percent
            100,  # Kicker Charge Percent
        )
        self._socket.sendto(packet, self._bridge_endpoint)

    def _runConnected(self):
        try:
            data = self._socket.recv(508)
            command_code = data[8]
            if command_code == 3:
                self._connected = False
                print('Disconnected by request.')
            elif command_code == 201:
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
