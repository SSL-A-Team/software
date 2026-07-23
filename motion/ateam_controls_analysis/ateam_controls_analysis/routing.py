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

"""
Pure, ROS-agnostic routing logic for controls analysis.

Given the per-sample contents of ``BodyControlExtendedTelemetry``, produce the
per-dimension field values that populate ``ControlsAnalysis.msg`` (three nested
``DimensionAnalysis`` sub-messages, one each for x, y and theta). Kept free of
any ROS/rclpy imports so it is trivially unit-testable.

Conventions:
  * NaN marks 'no data at this instant' (creates gaps in PlotJuggler).
  * The software command (``cmd_echo``) is routed onto the derivative implied
    by the active body control mode, and rotated local->global when needed.
  * Reference trajectory and command are additionally emitted as per-mode
    series (non-NaN only while that mode is active) for color-by-mode plots.
"""

from dataclasses import dataclass
import math
from typing import Dict, Optional, Sequence, Tuple

NAN = float('nan')

# Body control mode values (mirror BodyControlExtendedTelemetry.msg).
BCM_OFF = 0
BCM_ESTOP_BRAKE = 1
BCM_GLOBAL_POSITION = 10
BCM_GLOBAL_VELOCITY = 11
BCM_LOCAL_VELOCITY = 12
BCM_GLOBAL_ACCEL = 13
BCM_LOCAL_ACCEL = 14
BCM_HEADING_PIVOT = 20
BCM_POINT_PIVOT = 21
BCM_HEADING_LINE = 30
BCM_POINT_LINE = 31

# Dimension index into the telemetry float arrays.
DIMS = ('x', 'y', 'theta')
DIM_INDEX = {'x': 0, 'y': 1, 'theta': 2}

# Short mode names used to build per-mode split-series field names.
MODE_SUFFIX = {
    BCM_GLOBAL_POSITION: 'global_pos',
    BCM_GLOBAL_VELOCITY: 'global_vel',
    BCM_LOCAL_VELOCITY: 'local_vel',
    BCM_GLOBAL_ACCEL: 'global_acc',
    BCM_LOCAL_ACCEL: 'local_acc',
    BCM_HEADING_PIVOT: 'heading_pivot',
    BCM_POINT_PIVOT: 'point_pivot',
    BCM_HEADING_LINE: 'heading_line',
    BCM_POINT_LINE: 'point_line',
}

# Modes for which the firmware computes trajectory setpoints (bang-bang);
# used to color the reference-trajectory curves on the position/velocity plots.
TRAJ_MODES = (
    BCM_GLOBAL_POSITION,
    BCM_GLOBAL_VELOCITY,
    BCM_LOCAL_VELOCITY,
    BCM_HEADING_PIVOT,
    BCM_POINT_PIVOT,
    BCM_HEADING_LINE,
    BCM_POINT_LINE,
)

# mode -> (derivative name, is_local_frame) for the software command.
_CMD_ROUTE = {
    BCM_GLOBAL_POSITION: ('pos', False),
    BCM_GLOBAL_VELOCITY: ('vel', False),
    BCM_LOCAL_VELOCITY: ('vel', True),
    BCM_GLOBAL_ACCEL: ('accel', False),
    BCM_LOCAL_ACCEL: ('accel', True),
}


@dataclass
class TelemetrySample:
    """The subset of ExtendedTelemetry needed for analysis, as plain floats."""

    mode: int
    kf_pos_estimate: Sequence[float]        # [x, y, theta]
    kf_vel_estimate: Sequence[float]        # [x, y, theta]
    traj_pos: Sequence[float]               # [x, y, theta]
    traj_vel: Sequence[float]               # [x, y, theta]
    vision_pose: Sequence[float]            # [x, y, theta]
    imu_gyro: Sequence[float]               # [x, y, z]
    imu_accel: Sequence[float]              # [x, y, z]
    accel_u: Sequence[float]                # [x, y, theta]
    accel_u_fric_comp: Sequence[float]      # [x, y, theta]
    # Active maneuver's echoed command in its native frame, or None if the
    # active mode has no direct x/y/theta mapping (OFF, ESTOP, pivot, line).
    cmd_native: Optional[Tuple[float, float, float]]


# Default backward jump (microseconds) in the robot clock that is treated as a
# reboot. A reboot resets the counter to near zero, so the drop is many seconds;
# 1 s comfortably exceeds any out-of-order/jitter on a ~60 Hz telemetry stream.
DEFAULT_REBOOT_RESET_THRESHOLD_US = 1_000_000


def robot_timestamp_us(hi: int, lo: int) -> int:
    """Reconstruct the 64-bit robot microsecond counter from its hi/lo words."""
    return (int(hi) << 32) | int(lo)


class RobotClock:
    """
    Map a robot's monotonic microsecond counter onto a ROS-grounded timeline.

    The first sample is grounded at its ROS receive time; subsequent samples
    advance by the robot's own elapsed time, so the reconstructed timeline
    reflects true robot time regardless of bag-playback rate. When the robot
    reboots (counter jumps backward by more than ``reset_threshold_us``), the
    timeline is re-grounded at that sample's ROS time and a reboot is reported.
    """

    def __init__(self, reset_threshold_us: int = DEFAULT_REBOOT_RESET_THRESHOLD_US):
        self.reset_threshold_us = reset_threshold_us
        self._first_ros_ns: Optional[int] = None
        self._first_robot_us: Optional[int] = None
        self._last_robot_us: Optional[int] = None
        self.reboot_count = 0

    def _ground(self, robot_us: int, ros_ns: int) -> None:
        self._first_ros_ns = ros_ns
        self._first_robot_us = robot_us

    def update(self, robot_us: int, ros_ns: int) -> Tuple[int, bool]:
        """
        Advance the clock with one sample.

        Returns ``(stamp_ns, reboot)`` where ``stamp_ns`` is the reconstructed
        timeline timestamp (ns) and ``reboot`` is True only on the first sample
        after a detected reboot.
        """
        reboot = False
        if self._first_ros_ns is None:
            self._ground(robot_us, ros_ns)
        elif robot_us + self.reset_threshold_us < self._last_robot_us:
            self.reboot_count += 1
            reboot = True
            self._ground(robot_us, ros_ns)
        self._last_robot_us = robot_us
        stamp_ns = self._first_ros_ns + (robot_us - self._first_robot_us) * 1000
        return stamp_ns, reboot


def dimension_field_names() -> Tuple[str, ...]:
    """Return every value field name in DimensionAnalysis.msg."""
    names = [
        'pos_estimate', 'pos_traj', 'pos_vision', 'pos_cmd',
        'vel_estimate', 'vel_traj', 'vel_cmd', 'vel_gyro',
        'accel_u', 'accel_u_fric_comp', 'accel_imu', 'accel_cmd',
    ]
    for deriv in ('pos', 'vel'):
        for m in TRAJ_MODES:
            names.append(f'{deriv}_traj_{MODE_SUFFIX[m]}')
    names += [
        'pos_cmd_global_pos',
        'vel_cmd_global_vel', 'vel_cmd_local_vel',
        'accel_cmd_global_acc', 'accel_cmd_local_acc',
    ]
    return tuple(names)


def _rotate_local_to_global(
    native: Tuple[float, float, float], theta: float
) -> Tuple[float, float, float]:
    """
    Rotate a local-frame planar command into the global frame.

    The x/y components rotate by ``theta``; the angular component is unchanged.
    """
    lx, ly, ang = native
    c, s = math.cos(theta), math.sin(theta)
    return (c * lx - s * ly, s * lx + c * ly, ang)


def compute_dimensions(
    sample: TelemetrySample, prev_mode: Optional[int] = None
) -> Dict[str, Dict[str, float]]:
    """
    Compute per-dimension DimensionAnalysis field values for one sample.

    Returns ``{dim: {field: value}}`` for dim in ('x', 'y', 'theta'), where the
    inner keys are DimensionAnalysis field names. ``prev_mode`` breaks the
    single-series command curves at mode transitions (one-sample NaN gap) so the
    convenience (non-color) command curves are also discontinuous on mode change.
    """
    field_names = dimension_field_names()
    result = {d: {name: NAN for name in field_names} for d in DIMS}

    theta_est = float(sample.kf_pos_estimate[2])

    # Command routing (shared across dims): resolve global-frame command tuple.
    route = _CMD_ROUTE.get(sample.mode)
    cmd_deriv = None
    cmd_global = None
    if route is not None and sample.cmd_native is not None:
        cmd_deriv, is_local = route
        cmd = tuple(float(v) for v in sample.cmd_native)
        cmd_global = _rotate_local_to_global(cmd, theta_est) if is_local else cmd
    mode_changed = prev_mode is not None and prev_mode != sample.mode

    for d in DIMS:
        i = DIM_INDEX[d]
        f = result[d]
        f['pos_estimate'] = float(sample.kf_pos_estimate[i])
        f['pos_traj'] = float(sample.traj_pos[i])
        f['pos_vision'] = float(sample.vision_pose[i])
        f['vel_estimate'] = float(sample.kf_vel_estimate[i])
        f['vel_traj'] = float(sample.traj_vel[i])
        f['accel_u'] = float(sample.accel_u[i])
        f['accel_u_fric_comp'] = float(sample.accel_u_fric_comp[i])

        # Measurements on their physically correct derivative / dimension only.
        if d == 'theta':
            f['vel_gyro'] = float(sample.imu_gyro[2])
        else:
            f['accel_imu'] = float(sample.imu_accel[i])

        # Per-mode reference-trajectory split series (for color-by-mode).
        if sample.mode in TRAJ_MODES:
            suffix = MODE_SUFFIX[sample.mode]
            f[f'pos_traj_{suffix}'] = float(sample.traj_pos[i])
            f[f'vel_traj_{suffix}'] = float(sample.traj_vel[i])

        # Software command (single-series + per-mode split series).
        if cmd_global is not None:
            suffix = MODE_SUFFIX[sample.mode]
            if not mode_changed:
                f[f'{cmd_deriv}_cmd'] = cmd_global[i]
            f[f'{cmd_deriv}_cmd_{suffix}'] = cmd_global[i]

    return result
