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

import socket
from typing import Text

import launch
from launch import LaunchContext, Substitution
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.utilities import normalize_to_list_of_substitutions


class InterfaceFromAddressSubstitution(Substitution):
    """Substitution that returns the interface address to use for a given IP address."""

    def __init__(self, address: SomeSubstitutionsType) -> None:
        """Initialize the substitution."""
        super().__init__()
        self.__address = normalize_to_list_of_substitutions(address)

    def perform(self, context: LaunchContext) -> Text:
        """
        Perform the substitution, converting the address to an interface address.

        If the address given is empty, this returns an empty string.
        """
        address = context.perform_substitution(self.__address[0])
        return self.get_local_ip(address)

    def get_local_ip(self, target_ip: str, target_port: int = 80) -> str:
        """Return the local IP of the interface that would be used to reach target_ip."""
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            # UDP "connect" doesn't send any packets — it just asks the OS
            # to pick a route/interface, which lets us read the local IP.
            s.connect((target_ip, target_port))
            local_ip = s.getsockname()[0]
        except Exception:
            launch.logging.get_logger().exception(
                f'Could not determine interface for {target_ip}')
            raise
        finally:
            s.close()
        return local_ip
