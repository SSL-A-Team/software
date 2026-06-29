#!/usr/bin/python3

# Copyright 2026 A Team
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

"""Hardcode the SSL game-controller's blue team name on launch.

The ssl-game-controller has no config field or CLI flag for team names; they can
only be set through the UI or its API. This one-shot script connects to the GC's
``/api/control`` WebSocket and sends a single ``UpdateTeamState`` change setting the
blue team's name (default "A-Team"). Our ``team_client_node`` registers under that
same name, so once the blue slot is named to match, the GC binds our team to blue
automatically without any manual UI interaction.

Uses only the Python standard library (no websocket dependency): it performs the
HTTP Upgrade handshake and writes a single masked text frame by hand.
"""

import argparse
import base64
import json
import os
import socket
import struct
import sys
import time


def _build_payload(team_name):
    """Build the GC API ``Input`` JSON that sets the blue team name.

    protojson accepts lowerCamelCase field names and string enum values on input.
    """
    return json.dumps({
        'change': {
            'updateTeamStateChange': {
                'forTeam': 'BLUE',
                'teamName': team_name,
            }
        }
    })


def _send_ws_message(address, port, message, connect_timeout):
    """Open a WebSocket to the GC, send one text frame, and close.

    Returns True on success. Raises OSError if the TCP connection fails (so the
    caller can retry) and RuntimeError if the HTTP upgrade is rejected.
    """
    sock = socket.create_connection((address, port), timeout=connect_timeout)
    try:
        sock.settimeout(connect_timeout)

        key = base64.b64encode(os.urandom(16)).decode('ascii')
        handshake = (
            'GET /api/control HTTP/1.1\r\n'
            'Host: {host}:{port}\r\n'
            'Upgrade: websocket\r\n'
            'Connection: Upgrade\r\n'
            'Sec-WebSocket-Key: {key}\r\n'
            'Sec-WebSocket-Version: 13\r\n'
            '\r\n'
        ).format(host=address, port=port, key=key)
        sock.sendall(handshake.encode('ascii'))

        response = _read_http_response(sock)
        status_line = response.split('\r\n', 1)[0]
        if '101' not in status_line:
            raise RuntimeError(
                'GC rejected WebSocket upgrade: {}'.format(status_line))

        sock.sendall(_encode_text_frame(message))
        # Give the GC a moment to process before we close the socket.
        time.sleep(0.1)
        return True
    finally:
        try:
            sock.close()
        except OSError:
            pass


def _read_http_response(sock):
    """Read the HTTP handshake response up to the end of headers."""
    data = b''
    while b'\r\n\r\n' not in data:
        chunk = sock.recv(1024)
        if not chunk:
            break
        data += chunk
        if len(data) > 65536:
            break
    return data.decode('latin-1', errors='replace')


def _encode_text_frame(message):
    """Encode ``message`` as a single masked client-to-server text frame."""
    payload = message.encode('utf-8')
    # FIN=1, opcode=0x1 (text)
    header = bytearray([0x81])

    length = len(payload)
    mask_bit = 0x80
    if length < 126:
        header.append(mask_bit | length)
    elif length < 65536:
        header.append(mask_bit | 126)
        header += struct.pack('>H', length)
    else:
        header.append(mask_bit | 127)
        header += struct.pack('>Q', length)

    mask_key = os.urandom(4)
    header += mask_key
    masked = bytes(b ^ mask_key[i % 4] for i, b in enumerate(payload))
    return bytes(header) + masked


def _main(argv=None):
    parser = argparse.ArgumentParser(
        description='Set the SSL game-controller blue team name via its API.')
    parser.add_argument('--team-name', default='A-Team',
                        help='Team name to assign to the blue team (default: A-Team).')
    parser.add_argument('--gc-address', default='localhost',
                        help='Game-controller API host (default: localhost).')
    parser.add_argument('--gc-port', type=int, default=8081,
                        help='Game-controller API port (default: 8081).')
    parser.add_argument('--timeout', type=float, default=30.0,
                        help='Seconds to keep retrying until the GC is reachable '
                             '(default: 30).')
    args = parser.parse_args(argv)

    payload = _build_payload(args.team_name)
    deadline = time.monotonic() + args.timeout
    attempt = 0
    last_error = None

    while time.monotonic() < deadline:
        attempt += 1
        try:
            _send_ws_message(args.gc_address, args.gc_port, payload,
                            connect_timeout=2.0)
            print('[set_gc_team_name] Set blue team name to "{}" via {}:{}'.format(
                args.team_name, args.gc_address, args.gc_port))
            return 0
        except OSError as exc:
            # GC likely not up yet; wait and retry.
            last_error = exc
            time.sleep(1.0)
        except RuntimeError as exc:
            last_error = exc
            time.sleep(1.0)

    print('[set_gc_team_name] Failed to set blue team name after {} attempt(s): {}'
          .format(attempt, last_error), file=sys.stderr)
    return 1


if __name__ == '__main__':
    sys.exit(_main())
