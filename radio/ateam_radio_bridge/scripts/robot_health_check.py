#!/usr/bin/env python3
"""Passive robot link/health check. Never publishes motion commands.

Run with the radio bridge up (e.g. bringup with launch_kenobi:=false and no
joystick), or pass --launch-bridge to start a bare radio_bridge_node. With no
command publisher the bridge only sends heartbeats, so the robot stays idle.

  ./robot_health_check.py --robot 3 [--duration 10] [--launch-bridge]

Exit code is 0 only if every check passes.
"""

import argparse
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node

from ateam_radio_msgs.msg import BasicTelemetry, ConnectionStatus, ErrorTelemetry, ExtendedTelemetry

ERROR_FLAGS = [
    "power_error", "power_board_error", "battery_error", "battery_low", "battery_crit",
    "shutdown_pending", "tipped_error", "breakbeam_error",
    "accelerometer_0_error", "accelerometer_1_error", "gyroscope_0_error", "gyroscope_1_error",
    "motor_fl_general_error", "motor_fl_hall_error", "motor_bl_general_error",
    "motor_bl_hall_error", "motor_br_general_error", "motor_br_hall_error",
    "motor_fr_general_error", "motor_fr_hall_error", "motor_drib_general_error",
    "motor_drib_hall_error", "kicker_board_error",
]


class HealthCheck(Node):
    def __init__(self, robot, bridge_node):
        super().__init__("robot_health_check")
        base = f"/{bridge_node}/robot_feedback"
        self.connected_samples = []
        self.basic = []
        self.extended_count = 0
        self.errors = []
        self.create_subscription(
            ConnectionStatus, f"{base}/connection/robot{robot}",
            lambda m: self.connected_samples.append(m.radio_connected), 10)
        self.create_subscription(
            BasicTelemetry, f"{base}/basic/robot{robot}",
            lambda m: self.basic.append(m), 100)
        self.create_subscription(
            ExtendedTelemetry, f"{base}/extended/robot{robot}",
            lambda m: setattr(self, "extended_count", self.extended_count + 1), 100)
        self.create_subscription(
            ErrorTelemetry, f"{base}/error/robot{robot}",
            lambda m: self.errors.append(bytes(m.error_message).decode(errors="replace")), 10)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", type=int, required=True)
    ap.add_argument("--duration", type=float, default=10.0, help="sample window (s)")
    ap.add_argument("--connect-timeout", type=float, default=30.0)
    ap.add_argument("--min-rate", type=float, default=20.0, help="min basic telemetry Hz")
    ap.add_argument("--max-loss", type=float, default=0.2, help="max sequence-gap loss fraction")
    ap.add_argument("--min-battery", type=int, default=15)
    ap.add_argument("--bridge-node", default="radio_bridge")
    ap.add_argument("--launch-bridge", action="store_true")
    args = ap.parse_args()

    bridge = None
    if args.launch_bridge:
        bridge = subprocess.Popen(
            ["ros2", "run", "ateam_radio_bridge", "radio_bridge_node"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    rclpy.init()
    node = HealthCheck(args.robot, args.bridge_node)
    results = []

    def check(name, ok, detail=""):
        results.append(ok)
        print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f": {detail}" if detail else ""))

    try:
        print(f"Waiting up to {args.connect_timeout:.0f}s for robot {args.robot}...")
        deadline = time.monotonic() + args.connect_timeout
        while time.monotonic() < deadline and not node.basic:
            rclpy.spin_once(node, timeout_sec=0.1)
        check("telemetry received", bool(node.basic),
              "" if node.basic else "none; robot not discovered (radio/WiFi/hash mismatch?)")
        if not node.basic:
            print("RESULT: FAIL")
            return 1

        node.basic.clear()
        node.extended_count = 0
        node.errors.clear()
        end = time.monotonic() + args.duration
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

        msgs = list(node.basic)
        check("bridge reports connected",
              bool(node.connected_samples) and node.connected_samples[-1],
              f"{len(node.connected_samples)} samples")
        rate = len(msgs) / args.duration
        check("basic telemetry rate", rate >= args.min_rate,
              f"{rate:.1f} Hz (min {args.min_rate})")

        if len(msgs) >= 2:
            seqs = [m.transmission_sequence_number for m in msgs]
            missed = sum(((b - a) % 256) - 1 for a, b in zip(seqs, seqs[1:]))
            loss = missed / (missed + len(msgs) - 1)
            check("sequence numbers advance", len(set(seqs)) > 1, f"{len(set(seqs))} unique")
            check("packet loss", loss <= args.max_loss,
                  f"{loss * 100:.1f}% (max {args.max_loss * 100:.0f}%)")

        if msgs:
            last = msgs[-1]
            check("robot idle (BCM_OFF)", last.body_control_mode == BasicTelemetry.BCM_OFF,
                  f"mode={last.body_control_mode}")
            flagged = sorted({f for m in msgs for f in ERROR_FLAGS if getattr(m, f)})
            check("no hardware error flags", not flagged, ", ".join(flagged))
            check("battery", last.battery_percent >= args.min_battery,
                  f"{last.battery_percent}%")
        check("extended telemetry", node.extended_count > 0, f"{node.extended_count} msgs")
        check("no ErrorTelemetry", not node.errors, "; ".join(node.errors[:3]))
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if bridge:
            bridge.terminate()

    ok = all(results)
    print("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
