#!/usr/bin/env python3
"""
ROS2 node wrapping a BLE connection to a RoboCupJunior Soccer Communication
Module (Nordic UART Service based) and exposing ROS services to trigger the
"start" (PLAY) and "stop" (STOP) commands.

Protocol reference:
https://github.com/robocup-junior/soccer-communication-module/blob/master/docs/ai/03_ble_protocol.md

- The module is a BLE peripheral using the Nordic UART Service (NUS) UUIDs.
- Commands are written to the RX characteristic as: byte[0] = msg_id, byte[1..] = payload.
- BLE_MSG_PLAY = 4, no payload -> just write a single byte: 0x04
- BLE_MSG_STOP = 5, no payload -> just write a single byte: 0x05
- Devices advertise as "RCJs-m_<MAC>" (e.g. "RCJs-m_AA:BB:CC:DD:EE:FF").

Services exposed (std_srvs/srv/Trigger):
    ~/play  -> connects (if needed) and sends BLE_MSG_PLAY, retrying until it succeeds
    ~/stop  -> connects (if needed) and sends BLE_MSG_STOP, retrying until it succeeds

Since rclpy callbacks are synchronous but bleak is asyncio-based, this node
runs a dedicated asyncio event loop on a background thread and marshals BLE
calls onto it via asyncio.run_coroutine_threadsafe(), blocking the service
callback until the BLE operation completes.

Connect/send is retried indefinitely (with a delay between attempts) until
it succeeds, so ~/play and ~/stop only return once the command has actually
been delivered (or the node is shutting down).

Requires: pip install bleak
          ROS2 (rclpy, std_srvs)

Run:
    ros2 run <your_package> rcj_ble_node
    # or directly:
    python3 rcj_ble_node.py --ros-args -p device_address:=AA:BB:CC:DD:EE:FF

Trigger from the command line:
    ros2 service call /rcj_ble_node/play std_srvs/srv/Trigger {}
    ros2 service call /rcj_ble_node/stop std_srvs/srv/Trigger {}
"""

import asyncio
import threading

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from bleak import BleakClient, BleakScanner

# NUS UUIDs from the protocol doc
RX_CHAR_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  # app -> module (write)
TX_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  # module -> app (notify)

# Message IDs (ble_msg_id enum ordinals -- do not change order/values)
BLE_MSG_PLAY = 4
BLE_MSG_STOP = 5

DEVICE_NAME_PREFIX = "RCJs-m_"

SCAN_TIMEOUT_S = 1.0
BLE_OP_TIMEOUT_S = 15.0
RETRY_DELAY_S = 1.0


class RcjBleNode(Node):
    def __init__(self):
        super().__init__("rcj_ble_node")

        self.declare_parameter("device_address", "")
        self.declare_parameter("scan_timeout", SCAN_TIMEOUT_S)
        self.declare_parameter("keep_connected", True)
        self.declare_parameter("retry_delay", RETRY_DELAY_S)

        self._device_address = (
            self.get_parameter("device_address").get_parameter_value().string_value
            or None
        )
        self._scan_timeout = (
            self.get_parameter("scan_timeout").get_parameter_value().double_value
        )
        self._keep_connected = (
            self.get_parameter("keep_connected").get_parameter_value().bool_value
        )
        self._retry_delay = (
            self.get_parameter("retry_delay").get_parameter_value().double_value
        )

        self._client: BleakClient | None = None
        self._client_lock = asyncio.Lock()
        # Set on shutdown so any in-progress retry loop stops promptly
        # instead of retrying forever.
        self._shutting_down = False

        # --- background asyncio event loop, since bleak is async-only ---
        self._loop = asyncio.new_event_loop()
        self._loop_thread = threading.Thread(
            target=self._run_event_loop, daemon=True
        )
        self._loop_thread.start()

        # --- ROS services ---
        self._play_srv = self.create_service(
            Trigger, "~/play", self._handle_play
        )
        self._stop_srv = self.create_service(
            Trigger, "~/stop", self._handle_stop
        )

        self.get_logger().info(
            "rcj_ble_node ready. Call ~/play or ~/stop to send BLE commands."
        )

    def _run_event_loop(self):
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    def _run_coro(self, coro, timeout: "float | None" = BLE_OP_TIMEOUT_S):
        """Run a coroutine on the background loop and block for the result.

        timeout=None blocks until the coroutine finishes, however long that
        takes (used for the retrying send, which by design keeps trying
        until it succeeds or the node is shutting down).
        """
        future = asyncio.run_coroutine_threadsafe(coro, self._loop)
        return future.result(timeout=timeout)

    # ------------------------------------------------------------------
    # BLE helpers (run on the background event loop)
    # ------------------------------------------------------------------
    async def _find_robot(self):
        self.get_logger().info(
            f"Scanning for BLE devices for up to {self._scan_timeout}s..."
        )
        devices = await BleakScanner.discover(timeout=self._scan_timeout)
        for d in devices:
            if d.name and d.name.startswith(DEVICE_NAME_PREFIX):
                self.get_logger().info(f"Found robot: {d.name} ({d.address})")
                return d
        return None

    async def _ensure_connected(self) -> BleakClient:
        async with self._client_lock:
            if self._client is not None and self._client.is_connected:
                return self._client

            address = self._device_address
            if address is None:
                device = await self._find_robot()
                if device is None:
                    raise RuntimeError(
                        "No robot found. Make sure it's powered on and advertising."
                    )
                address = device.address

            self.get_logger().info(f"Connecting to {address}...")
            client = BleakClient(address)
            await client.connect()
            if not client.is_connected:
                raise RuntimeError("Failed to connect.")

            self._client = client
            self.get_logger().info("Connected.")
            return client

    async def _disconnect_if_transient(self, client: BleakClient):
        if not self._keep_connected:
            await client.disconnect()
            self._client = None

    async def _send_message(self, msg_id: int, label: str):
        """Connect (if needed) and write the message, retrying forever
        (with a delay between attempts) until it succeeds or the node is
        shutting down."""
        message = bytes([msg_id])
        attempt = 0
        while not self._shutting_down:
            attempt += 1
            try:
                client = await self._ensure_connected()
                self.get_logger().info(
                    f"Sending {label} command (attempt {attempt}): {message.hex()}"
                )
                await client.write_gatt_char(RX_CHAR_UUID, message, response=False)
                await self._disconnect_if_transient(client)
                return
            except Exception as exc:  # noqa: BLE001 - retry on any BLE error
                self.get_logger().warn(
                    f"Attempt {attempt} to send {label} failed: {exc}. "
                    f"Retrying in {self._retry_delay}s..."
                )
                # Drop any stale/half-open client so the next attempt
                # starts with a fresh connect (and, if no fixed address
                # was configured, a fresh scan).
                async with self._client_lock:
                    stale_client = self._client
                    self._client = None
                if stale_client is not None:
                    try:
                        await stale_client.disconnect()
                    except Exception:
                        pass
                await asyncio.sleep(self._retry_delay)

        raise RuntimeError(f"{label} send cancelled: node is shutting down.")

    # ------------------------------------------------------------------
    # ROS service callbacks
    # ------------------------------------------------------------------
    def _handle_play(self, request, response):
        try:
            # timeout=None: block as long as it takes, since _send_message
            # itself retries indefinitely until it succeeds.
            self._run_coro(
                self._send_message(BLE_MSG_PLAY, "start (PLAY)"), timeout=None
            )
            response.success = True
            response.message = "PLAY command sent."
        except Exception as exc:  # noqa: BLE001 - surface any BLE error to the caller
            self.get_logger().error(f"Failed to send PLAY: {exc}")
            response.success = False
            response.message = str(exc)
        return response

    def _handle_stop(self, request, response):
        try:
            self._run_coro(
                self._send_message(BLE_MSG_STOP, "stop (STOP)"), timeout=None
            )
            response.success = True
            response.message = "STOP command sent."
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Failed to send STOP: {exc}")
            response.success = False
            response.message = str(exc)
        return response

    # ------------------------------------------------------------------
    def destroy_node(self):
        # Tell any in-progress retry loop (in _send_message) to give up
        # instead of retrying forever.
        self._shutting_down = True
        if self._client is not None:
            try:
                self._run_coro(self._client.disconnect(), timeout=5.0)
            except Exception:
                pass
        self._loop.call_soon_threadsafe(self._loop.stop)
        self._loop_thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RcjBleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
