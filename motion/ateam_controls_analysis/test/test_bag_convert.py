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
End-to-end round-trip test for the bag -> bag converter.

Writes a tiny input bag of synthetic ``ExtendedTelemetry`` messages (covering a
mode change and a robot reboot), runs :func:`convert_bag`, then reads the output
bag back and asserts the ``ControlsAnalysis`` timestamps and key fields.
"""

import math

from ateam_controls_analysis.bag_convert import (
    available_robot_ids,
    CONTROLS_ANALYSIS_TYPE,
    convert_bag,
    default_input_topic,
    default_output_uri,
    main,
    robot_id_from_topic,
)
from ateam_controls_analysis.routing import (
    BCM_GLOBAL_VELOCITY,
    BCM_LOCAL_VELOCITY,
)
from ateam_controls_analysis_msgs.msg import ControlsAnalysis
from ateam_radio_msgs.msg import ExtendedTelemetry
from rclpy.serialization import deserialize_message, serialize_message
import rosbag2_py

ROBOT_ID = 2
INPUT_TOPIC = default_input_topic(ROBOT_ID)
STORAGE_ID = 'mcap'


def _make_extended(robot_us, mode, vel_cmd, theta=0.0):
    """Build a minimal ExtendedTelemetry with the fields the routing reads."""
    msg = ExtendedTelemetry()
    msg.timestamp_us_lo = robot_us & 0xFFFFFFFF
    msg.timestamp_us_hi = (robot_us >> 32) & 0xFFFFFFFF
    bct = msg.body_control_telemetry
    bct.body_control_mode = mode
    # Real telemetry always carries length-3 arrays; populate every array the
    # routing reads so a synthetic message matches on-robot shape.
    bct.kf_body_pos_estimate = [0.0, 0.0, float(theta)]
    bct.kf_body_vel_estimate = [0.0, 0.0, 0.0]
    bct.body_traj_pos = [0.0, 0.0, 0.0]
    bct.body_traj_vel = [0.0, 0.0, 0.0]
    bct.vision_pose = [0.0, 0.0, 0.0]
    bct.imu_gyro = [0.0, 0.0, 0.0]
    bct.imu_accel = [0.0, 0.0, 0.0]
    bct.body_accel_u = [0.0, 0.0, 0.0]
    bct.body_accel_u_fric_comp = [0.0, 0.0, 0.0]
    # Per-wheel motor telemetry. Give each wheel a distinct signature so the
    # Velocity/Current view extraction (incl. current = mean(current_samples))
    # can be asserted. front_left carries a two-sample current buffer whose mean
    # is 15.0 with a positive setpoint; back_right carries the same magnitude
    # buffer but a negative setpoint, so its measured current must flip sign to
    # -15.0. The remaining wheels are left with empty buffers (current -> NaN).
    msg.front_left_motor.velocity_telemetry.wheel_vel_rads = 1.0
    msg.front_left_motor.velocity_telemetry.vel_setpoint_rads = 1.5
    msg.front_left_motor.current_telemetry.current_setpoint_ma = 200
    msg.front_left_motor.current_telemetry.current_samples_ma = [10, 20]
    msg.back_right_motor.current_telemetry.current_setpoint_ma = -200
    msg.back_right_motor.current_telemetry.current_samples_ma = [10, 20]
    if mode == BCM_GLOBAL_VELOCITY:
        e = bct.maneuver_global_vel.cmd_echo
        e.global_xd, e.global_yd, e.global_omega = vel_cmd
    elif mode == BCM_LOCAL_VELOCITY:
        e = bct.maneuver_local_vel.cmd_echo
        e.local_xd, e.local_yd, e.local_omega = vel_cmd
    return msg


def _write_input_bag(uri):
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=uri, storage_id=STORAGE_ID),
        rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'),
    )
    writer.create_topic(rosbag2_py.TopicMetadata(
        id=0, name=INPUT_TOPIC,
        type='ateam_radio_msgs/msg/ExtendedTelemetry',
        serialization_format='cdr'))

    # (bag_time_ns, robot_us, mode, vel_cmd, theta)
    # Global vel, then a mode change to local vel, then a reboot (robot_us drops).
    samples = [
        (1_000_000_000, 1_000_000, BCM_GLOBAL_VELOCITY, (1.0, 0.0, 0.0), 0.0),
        (1_100_000_000, 1_100_000, BCM_GLOBAL_VELOCITY, (1.0, 0.0, 0.0), 0.0),
        (1_200_000_000, 1_200_000, BCM_LOCAL_VELOCITY,
         (1.0, 0.0, 0.0), math.pi / 2),
        (5_000_000_000, 5_000, BCM_LOCAL_VELOCITY,
         (1.0, 0.0, 0.0), math.pi / 2),
    ]
    for bag_ns, robot_us, mode, vel, theta in samples:
        msg = _make_extended(robot_us, mode, vel, theta)
        writer.write(INPUT_TOPIC, serialize_message(msg), bag_ns)
    del writer
    return samples


def _read_output_bag(uri, topic):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=uri, storage_id=STORAGE_ID),
        rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    out = []
    while reader.has_next():
        t, data, stamp_ns = reader.read_next()
        if t == topic:
            out.append((stamp_ns, deserialize_message(data, ControlsAnalysis)))
    reader.close()
    return types, out


def test_convert_bag_round_trip(tmp_path):
    input_uri = str(tmp_path / 'in')
    output_uri = str(tmp_path / 'out')
    samples = _write_input_bag(input_uri)

    count = convert_bag(
        input_uri=input_uri, output_uri=output_uri, robot_id=ROBOT_ID,
        use_robot_time=True, output_storage_id=STORAGE_ID)
    assert count == len(samples)

    types, msgs = _read_output_bag(output_uri, '/controls_analysis')
    assert types['/controls_analysis'] == CONTROLS_ANALYSIS_TYPE
    assert len(msgs) == len(samples)

    # Robot-time axis: grounded at first bag time (1e9 ns), then advanced by
    # robot-elapsed time; bag timestamp and header stamp agree.
    first_stamp, first_msg = msgs[0]
    assert first_stamp == 1_000_000_000
    assert first_msg.body_control_mode == BCM_GLOBAL_VELOCITY
    # global vel cmd -> vel_cmd on x, no rotation.
    assert math.isclose(first_msg.x.vel_cmd, 1.0, rel_tol=1e-5)

    # Wheel telemetry (Velocity / Current views): front_left carries a distinct
    # signature; current is the mean of its current-sample buffer ([10, 20]).
    assert math.isclose(first_msg.front_left.vel, 1.0, rel_tol=1e-5)
    assert math.isclose(first_msg.front_left.vel_setpoint, 1.5, rel_tol=1e-5)
    assert math.isclose(first_msg.front_left.current, 15.0, rel_tol=1e-5)
    assert math.isclose(first_msg.front_left.current_setpoint, 200.0,
                        rel_tol=1e-5)
    # A negative commanded current setpoint flips the sign of the (magnitude)
    # measured current: same |mean| of 15.0, reported as -15.0.
    assert math.isclose(first_msg.back_right.current, -15.0, rel_tol=1e-5)
    assert math.isclose(first_msg.back_right.current_setpoint, -200.0,
                        rel_tol=1e-5)
    # Wheels with an empty current-sample buffer gap the current curve (NaN).
    assert math.isnan(first_msg.back_left.current)

    # Second sample advanced by 100 ms of robot time from the ground.
    assert msgs[1][0] == 1_000_000_000 + 100_000 * 1000

    # Mode change to local vel with theta = pi/2. The single-series command is
    # intentionally broken (NaN) at a mode transition; the per-mode split series
    # carries the rotated value (x cmd rotates onto +y).
    _, third = msgs[2]
    assert third.body_control_mode == BCM_LOCAL_VELOCITY
    assert math.isnan(third.x.vel_cmd)  # single-series gap at transition
    assert abs(third.y.vel_cmd_local_vel - 1.0) < 1e-4
    assert abs(third.x.vel_cmd_local_vel) < 1e-4

    # Reboot sample: robot_us dropped far below previous -> re-grounded at its
    # own bag time, and reboot_count increments. Mode unchanged from prev, so
    # the single-series command is populated and shows the local->global
    # rotation.
    reboot_stamp, reboot_msg = msgs[3]
    assert reboot_stamp == 5_000_000_000
    assert reboot_msg.reboot_count == 1
    assert abs(reboot_msg.y.vel_cmd - 1.0) < 1e-4
    assert abs(reboot_msg.x.vel_cmd) < 1e-4


def test_convert_bag_receive_time(tmp_path):
    input_uri = str(tmp_path / 'in')
    output_uri = str(tmp_path / 'out')
    _write_input_bag(input_uri)

    convert_bag(
        input_uri=input_uri, output_uri=output_uri, robot_id=ROBOT_ID,
        use_robot_time=False, output_storage_id=STORAGE_ID)

    _, msgs = _read_output_bag(output_uri, '/controls_analysis')
    # With use_robot_time=False the original bag timestamps are preserved.
    stamps = [s for s, _ in msgs]
    assert stamps == [1_000_000_000, 1_100_000_000, 1_200_000_000, 5_000_000_000]


def _write_multi_robot_bag(uri, robot_ids, empty_robot_ids=()):
    """Write a bag with one ExtendedTelemetry sample per robot id.

    ``empty_robot_ids`` create the extended-telemetry topic but write no
    messages on it, so they should not count as available robots.
    """
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=uri, storage_id=STORAGE_ID),
        rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'),
    )
    all_ids = list(robot_ids) + list(empty_robot_ids)
    for i, robot_id in enumerate(all_ids):
        writer.create_topic(rosbag2_py.TopicMetadata(
            id=i, name=default_input_topic(robot_id),
            type='ateam_radio_msgs/msg/ExtendedTelemetry',
            serialization_format='cdr'))
    for robot_id in robot_ids:
        msg = _make_extended(1_000_000, BCM_GLOBAL_VELOCITY, (1.0, 0.0, 0.0))
        writer.write(
            default_input_topic(robot_id), serialize_message(msg),
            1_000_000_000)
    del writer


def test_robot_id_from_topic():
    assert robot_id_from_topic('/robot_feedback/extended/robot0') == 0
    assert robot_id_from_topic('/robot_feedback/extended/robot13') == 13
    assert robot_id_from_topic('/robot_feedback/extended/robotX') is None
    assert robot_id_from_topic('/some/other/topic') is None


def test_available_robot_ids(tmp_path):
    input_uri = str(tmp_path / 'in')
    # Robot 5's topic exists but carries no messages -> excluded.
    _write_multi_robot_bag(input_uri, [3, 0, 7], empty_robot_ids=[5])
    assert available_robot_ids(input_uri) == [0, 3, 7]


def test_default_output_uri():
    assert default_output_uri('/path/to/bag', 4) == '/path/to/bag_robot4'
    assert default_output_uri('/path/to/bag/', 4) == '/path/to/bag_robot4'


def test_main_converts_all_robots_by_default(tmp_path):
    input_uri = str(tmp_path / 'game_bag')
    _write_multi_robot_bag(input_uri, [1, 4])

    # No --robot-id: every robot in the bag is converted to its own bag.
    main([input_uri, '--output-storage-id', STORAGE_ID])

    for robot_id in (1, 4):
        output_uri = default_output_uri(input_uri, robot_id)
        assert output_uri == f'{input_uri}_robot{robot_id}'
        types, msgs = _read_output_bag(output_uri, '/controls_analysis')
        assert types['/controls_analysis'] == CONTROLS_ANALYSIS_TYPE
        assert len(msgs) == 1
