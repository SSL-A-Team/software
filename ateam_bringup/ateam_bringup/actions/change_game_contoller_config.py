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

import launch
from launch.some_substitutions_type import SomeSubstitutionsType
from launch import LaunchContext, Substitution
from launch.actions import OpaqueCoroutine
from launch.utilities import normalize_to_list_of_substitutions
from typing import Mapping, Union, Any

import asyncio
import json
import websockets


class ChangeGameControllerConfig(OpaqueCoroutine):
    """Action that sends config delta commands to the GC API"""

    def __init__(self, configs: Mapping[Union[Substitution, str], Any], gc_address: SomeSubstitutionsType) -> None:
        """Initialize the action."""
        super().__init__(coroutine=self.my_coroutine)
        self.__configs = configs
        self.__gc_address = normalize_to_list_of_substitutions(gc_address)

    async def my_coroutine(self, context: LaunchContext, *args, **kwargs):
        gc_address = context.perform_substitution(self.__gc_address[0])
        config_delta = {}
        for k, v in self.__configs.items():
            if isinstance(k, Substitution):
                key = context.perform_substitution(k)
            else:
                key = k
            if isinstance(v, Substitution):
                value = context.perform_substitution(v)
            else:
                value = v
            config_delta[key] = value
        await self.send_json_payload(gc_address, config_delta)

    async def send_json_payload(self, address: str, configs: Mapping[str, str]):
        payload = {
            'config_delta': configs
        }
        server_url = f'ws://{address}:8081/api/control'
        while True:
            try:
                async with websockets.connect(server_url) as ws:
                    await ws.send(json.dumps(payload))
                    # await ws.recv()
                break
            except ConnectionRefusedError:
                continue
