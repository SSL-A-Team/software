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
Adapters from ``ateam_radio_msgs/ExtendedTelemetry`` to routing inputs.

This is the single place that knows how to read the fields of an
``ExtendedTelemetry`` message: it selects the active maneuver's echoed command
(``cmd_echo``) by body control mode and packs the per-sample telemetry into a
``TelemetrySample`` for the pure routing logic in :mod:`.routing`.

The ``maneuver`` field of ``BodyControlExtendedTelemetry`` is a C union
flattened by the message generator into one field per mode
(``maneuver_global_pos``, ``maneuver_global_vel``, ...). Only the field matching
the active ``body_control_mode`` holds valid data; the others alias the same
bytes. :func:`extract_cmd_native` therefore reads only the active field.

Both the live republisher node and the offline bag converter import from here so
the message-unpacking logic exists in exactly one place.
"""

from ateam_controls_analysis.routing import (
    BCM_GLOBAL_ACCEL,
    BCM_GLOBAL_POSITION,
    BCM_GLOBAL_VELOCITY,
    BCM_LOCAL_ACCEL,
    BCM_LOCAL_VELOCITY,
    DIMS,
    TelemetrySample,
)


def extract_cmd_native(bct):
    """
    Pull the active maneuver's echoed command as an ``(a, b, c)`` tuple.

    ``bct`` is a ``BodyControlExtendedTelemetry`` message. Returns ``None`` for
    modes without a direct x/y/theta command mapping (OFF, ESTOP, pivot, line).
    """
    mode = bct.body_control_mode
    if mode == BCM_GLOBAL_POSITION:
        e = bct.maneuver_global_pos.cmd_echo
        return (e.global_x, e.global_y, e.global_theta)
    if mode == BCM_GLOBAL_VELOCITY:
        e = bct.maneuver_global_vel.cmd_echo
        return (e.global_xd, e.global_yd, e.global_omega)
    if mode == BCM_LOCAL_VELOCITY:
        e = bct.maneuver_local_vel.cmd_echo
        return (e.local_xd, e.local_yd, e.local_omega)
    if mode == BCM_GLOBAL_ACCEL:
        e = bct.maneuver_global_acc.cmd_echo
        return (e.global_xdd, e.global_ydd, e.global_alpha)
    if mode == BCM_LOCAL_ACCEL:
        e = bct.maneuver_local_acc.cmd_echo
        return (e.local_xdd, e.local_ydd, e.local_alpha)
    return None


def sample_from_extended(msg):
    """Build a :class:`~.routing.TelemetrySample` from an ``ExtendedTelemetry``."""
    bct = msg.body_control_telemetry
    return TelemetrySample(
        mode=bct.body_control_mode,
        kf_pos_estimate=bct.kf_body_pos_estimate,
        kf_vel_estimate=bct.kf_body_vel_estimate,
        traj_pos=bct.body_traj_pos,
        traj_vel=bct.body_traj_vel,
        vision_pose=bct.vision_pose,
        imu_gyro=bct.imu_gyro,
        imu_accel=bct.imu_accel,
        accel_u=bct.body_accel_u,
        accel_u_fric_comp=bct.body_accel_u_fric_comp,
        cmd_native=extract_cmd_native(bct),
    )


def populate_analysis(out, sample, dims, reboot, reboot_count):
    """
    In-place fill a ``ControlsAnalysis`` message from routing outputs.

    ``out`` is a ``ControlsAnalysis`` message (its ``header`` must be stamped by
    the caller). ``sample`` is the :class:`~.routing.TelemetrySample` for this
    instant, ``dims`` is the ``{dim: {field: value}}`` mapping from
    :func:`~.routing.compute_dimensions`, and ``reboot`` / ``reboot_count`` come
    from :class:`~.routing.RobotClock`.
    """
    out.body_control_mode = int(sample.mode)
    out.theta_estimate = float(sample.kf_pos_estimate[2])
    out.reboot_event = 1.0 if reboot else 0.0
    out.reboot_count = int(reboot_count)
    for dim in DIMS:
        sub = getattr(out, dim)
        for name, value in dims[dim].items():
            setattr(sub, name, value)
    return out
