#! /usr/bin/python3
"""This script minimally emulates a robot for testing the radio bridge live."""

import argparse
import socket
import struct

parser = argparse.ArgumentParser()
parser.add_argument('--robot_id', type=int, default=0)
parser.add_argument('--color', type=str, choices=['blue', 'yellow'], default='blue')
parser.add_argument('--discovery_address', type=str, default='224.4.20.69')
parser.add_argument('--discovery_port', type=int, default=42069)
args = parser.parse_args()

robot_id = args.robot_id
color = args.color
discovery_endpoint = (args.discovery_address, args.discovery_port)
bridge_endpoint = ('', 0)
connected = False
telemetry_sequence_num = 0


def runDiscovery():
    """Send discovery requests and wait for a reply."""
    global bridge_endpoint
    global connected
    packet = struct.pack('IHHBHBB',
                         0,  # CRC
                         0,  # Version Major
                         1,  # Version Minor
                         101,  # CC Hello Req
                         2,  # Data Length
                         robot_id,  # Robot ID
                         1 if color == 'blue' else 0,  # Team Color
                         )
    sock.sendto(packet, discovery_endpoint)
    try:
        data = sock.recv(18)
        print(''.join('{:02x}'.format(x) for x in data))
        if len(data) < 18:
            print(f'Bad packet length: {len(data)}')
            return
        command_code = data[8]
        if command_code != 202:
            return
        bridge_port = (data[17] << 8) | data[16]
        bridge_ip = '.'.join([str(x) for x in data[12:16]])
        bridge_endpoint = (bridge_ip, bridge_port)
        connected = True
        print(f'Connected to radio bridge at {bridge_endpoint[0]}:{bridge_endpoint[1]}')
    except socket.timeout:
        pass


def sendTelemetryPacket():
    """Send a static telemetry packet."""
    global telemetry_sequence_num
    packet = struct.pack(
        'IHHBHHBBffIffffff',
        0,  # CRC
        0,  # Version Major
        1,  # Version Minor
        102,  # CC Telemetry
        40,  # Data Length
        telemetry_sequence_num,
        0,  # Robot Revision Major
        0,  # Robot Revision Minor
        24.0,  # Battery Voltage
        0.0,  # Battery Temperature
        0,  # Bit Flags
        0.0,  # Motor 0 Temp
        0.0,  # Motor 1 Temp
        0.0,  # Motor 2 Temp
        0.0,  # Motor 3 Temp
        0.0,  # Motor 4 Temp
        0.0,  # Kicker Charge Level
    )
    sock.sendto(packet, bridge_endpoint)
    telemetry_sequence_num += 1


def runConnected():
    """Receive commands and send feedback packets."""
    global connected
    try:
        data = sock.recv(508)
        command_code = data[8]
        if command_code == 3:
            connected = False
            print('Disconnected by request.')
        elif command_code == 201:
            sendTelemetryPacket()
    except socket.timeout:
        connected = False
        print('Disconnected by timeout.')


with socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as sock:
    sock.settimeout(1.0)
    print('Emulated robot running...')
    while True:
        if not connected:
            runDiscovery()
        else:
            runConnected()
