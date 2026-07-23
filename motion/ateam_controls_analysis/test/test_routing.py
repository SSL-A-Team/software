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

"""Unit tests for the pure controls-analysis routing logic."""

import math

from ateam_controls_analysis.routing import (
    BCM_GLOBAL_ACCEL,
    BCM_GLOBAL_POSITION,
    BCM_GLOBAL_VELOCITY,
    BCM_HEADING_PIVOT,
    BCM_LOCAL_ACCEL,
    BCM_LOCAL_VELOCITY,
    BCM_OFF,
    compute_dimensions,
    dimension_field_names,
    TelemetrySample,
)


def _sample(mode, cmd_native, theta=0.0):
    return TelemetrySample(
        mode=mode,
        kf_pos_estimate=[1.0, 2.0, theta],
        kf_vel_estimate=[0.1, 0.2, 0.3],
        traj_pos=[1.1, 2.1, theta + 0.01],
        traj_vel=[0.11, 0.21, 0.31],
        vision_pose=[1.2, 2.2, theta + 0.02],
        imu_gyro=[9.0, 9.0, 0.33],
        imu_accel=[0.5, 0.6, 9.81],
        accel_u=[0.01, 0.02, 0.03],
        accel_u_fric_comp=[0.011, 0.021, 0.031],
        cmd_native=cmd_native,
    )


def _isnan(v):
    return isinstance(v, float) and math.isnan(v)


def test_all_fields_present_and_nan_default():
    d = compute_dimensions(_sample(BCM_OFF, None))
    for dim in ('x', 'y', 'theta'):
        for name in dimension_field_names():
            assert name in d[dim]
        assert _isnan(d[dim]['pos_cmd'])
        assert _isnan(d[dim]['vel_cmd'])
        assert _isnan(d[dim]['accel_cmd'])


def test_estimate_traj_measurements_always_present():
    d = compute_dimensions(_sample(BCM_OFF, None))
    assert d['x']['pos_estimate'] == 1.0
    assert d['theta']['pos_estimate'] == 0.0
    assert d['x']['vel_estimate'] == 0.1
    assert d['x']['pos_traj'] == 1.1
    assert d['x']['vel_traj'] == 0.11
    assert d['x']['pos_vision'] == 1.2
    assert d['x']['accel_u'] == 0.01
    assert d['x']['accel_u_fric_comp'] == 0.011
    # measurements on correct derivative/dimension only
    assert d['theta']['vel_gyro'] == 0.33
    assert _isnan(d['x']['vel_gyro'])
    assert d['x']['accel_imu'] == 0.5
    assert d['y']['accel_imu'] == 0.6
    assert _isnan(d['theta']['accel_imu'])


def test_global_position_routes_to_position():
    d = compute_dimensions(_sample(BCM_GLOBAL_POSITION, (3.0, 4.0, 1.5)))
    assert d['x']['pos_cmd'] == 3.0
    assert d['y']['pos_cmd'] == 4.0
    assert d['theta']['pos_cmd'] == 1.5
    assert d['x']['pos_cmd_global_pos'] == 3.0
    assert _isnan(d['x']['vel_cmd'])
    assert _isnan(d['x']['accel_cmd'])


def test_global_velocity_routes_to_velocity():
    d = compute_dimensions(_sample(BCM_GLOBAL_VELOCITY, (0.5, -0.5, 0.2)))
    assert d['x']['vel_cmd'] == 0.5
    assert d['x']['vel_cmd_global_vel'] == 0.5
    assert _isnan(d['x']['pos_cmd'])


def test_local_velocity_rotated_to_global():
    d = compute_dimensions(_sample(BCM_LOCAL_VELOCITY, (1.0, 0.0, 0.7),
                                   theta=math.pi / 2))
    assert math.isclose(d['x']['vel_cmd'], 0.0, abs_tol=1e-9)
    assert math.isclose(d['y']['vel_cmd'], 1.0, abs_tol=1e-9)
    assert math.isclose(d['theta']['vel_cmd'], 0.7, abs_tol=1e-12)
    assert math.isclose(d['y']['vel_cmd_local_vel'], 1.0, abs_tol=1e-9)


def test_global_accel_routes_to_acceleration():
    d = compute_dimensions(_sample(BCM_GLOBAL_ACCEL, (2.0, 3.0, 0.9)))
    assert d['x']['accel_cmd'] == 2.0
    assert d['x']['accel_cmd_global_acc'] == 2.0
    assert _isnan(d['x']['pos_cmd'])
    assert _isnan(d['x']['vel_cmd'])


def test_local_accel_rotated():
    d = compute_dimensions(_sample(BCM_LOCAL_ACCEL, (0.0, 1.0, 0.4),
                                   theta=math.pi / 2))
    assert math.isclose(d['x']['accel_cmd'], -1.0, abs_tol=1e-9)
    assert math.isclose(d['y']['accel_cmd'], 0.0, abs_tol=1e-9)
    assert math.isclose(d['x']['accel_cmd_local_acc'], -1.0, abs_tol=1e-9)


def test_non_mapping_mode_gaps_command_but_keeps_traj_split():
    d = compute_dimensions(_sample(BCM_HEADING_PIVOT, None))
    for dim in ('x', 'y', 'theta'):
        assert _isnan(d[dim]['pos_cmd'])
        assert _isnan(d[dim]['vel_cmd'])
        assert _isnan(d[dim]['accel_cmd'])
    assert d['x']['pos_traj_heading_pivot'] == 1.1
    assert d['x']['vel_traj_heading_pivot'] == 0.11


def test_mode_change_breaks_single_series_command():
    d = compute_dimensions(_sample(BCM_GLOBAL_VELOCITY, (0.5, 0.0, 0.0)),
                           prev_mode=BCM_GLOBAL_POSITION)
    assert _isnan(d['x']['vel_cmd'])
    assert d['x']['vel_cmd_global_vel'] == 0.5


def test_traj_split_only_for_active_mode():
    d = compute_dimensions(_sample(BCM_GLOBAL_POSITION, (0.0, 0.0, 0.0)))
    assert d['x']['pos_traj_global_pos'] == 1.1
    assert _isnan(d['x']['pos_traj_global_vel'])
    assert _isnan(d['x']['pos_traj_point_line'])
