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
Offline bag -> bag converter for Foxglove.

Reads ``ateam_radio_msgs/ExtendedTelemetry`` from a recorded bag, runs the same
routing logic as the live republisher node, and writes a single NEW bag
containing the flat ``ateam_controls_analysis_msgs/ControlsAnalysis`` for every
robot, one topic per robot on ``/controls_analysis/robot{id}``. That output bag
can be opened directly in Foxglove (full-bag load) — no node, no
replay-streaming — and the bundled topic-alias extension lets you switch which
robot is displayed without reloading the bag.

Unlike the streaming node, no heartbeat is applied: for an offline full-bag load
pure NaN gaps are correct (there is no live buffer whose axis could stretch).
"""

import os
import re

from ateam_controls_analysis.routing import (
    compute_dimensions,
    DEFAULT_REBOOT_RESET_THRESHOLD_US,
    robot_timestamp_us,
    RobotClock,
)
from ateam_controls_analysis.telemetry import (
    populate_analysis,
    sample_from_extended,
)
from ateam_controls_analysis_msgs.msg import ControlsAnalysis
from ateam_radio_msgs.msg import ExtendedTelemetry
from rclpy.serialization import deserialize_message, serialize_message
from rclpy.time import Time
import rosbag2_py

CONTROLS_ANALYSIS_TYPE = 'ateam_controls_analysis_msgs/msg/ControlsAnalysis'

_EXTENDED_TOPIC_RE = re.compile(r'^/robot_feedback/extended/robot(\d+)$')


def default_input_topic(robot_id):
    """Return the default extended-telemetry topic for ``robot_id``."""
    return f'/robot_feedback/extended/robot{robot_id}'


def default_output_topic(robot_id):
    """Return the per-robot ControlsAnalysis output topic for ``robot_id``."""
    return f'/controls_analysis/robot{robot_id}'


def robot_id_from_topic(topic):
    """Return the robot id encoded in an extended-telemetry topic, or None."""
    match = _EXTENDED_TOPIC_RE.match(topic)
    return int(match.group(1)) if match else None


def available_robot_ids(input_uri, input_storage_id=''):
    """
    Return the sorted robot ids that have extended telemetry in the bag.

    Scans the input bag's topics for
    ``/robot_feedback/extended/robot{id}`` and returns the ids that carry at
    least one recorded message (topics present in the bag but empty are
    skipped).
    """
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=input_uri, storage_id=input_storage_id),
        rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'),
    )
    ids = set()
    for info in reader.get_metadata().topics_with_message_count:
        if info.message_count <= 0:
            continue
        robot_id = robot_id_from_topic(info.topic_metadata.name)
        if robot_id is not None:
            ids.add(robot_id)
    reader.close()
    return sorted(ids)


def default_output_uri(input_uri):
    """
    Return the default combined output bag path.

    Appends ``_controls_analysis`` to the input bag name; the single output bag
    holds every robot on ``/controls_analysis/robot{id}``.
    """
    return input_uri.rstrip('/') + '_controls_analysis'


def _open_reader(input_uri, input_topics, storage_id):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=input_uri, storage_id=storage_id),
        rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(input_topics)))
    return reader


def _open_writer(output_uri, output_topics, storage_id):
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=output_uri, storage_id=storage_id),
        rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'),
    )
    for i, topic in enumerate(output_topics):
        writer.create_topic(rosbag2_py.TopicMetadata(
            id=i, name=topic, type=CONTROLS_ANALYSIS_TYPE,
            serialization_format='cdr'))
    return writer


def convert_bag(input_uri, output_uri=None, robot_ids=None,
                use_robot_time=True,
                reboot_reset_threshold_us=DEFAULT_REBOOT_RESET_THRESHOLD_US,
                input_storage_id='', output_storage_id=None):
    """
    Convert a bag's ExtendedTelemetry into one combined ControlsAnalysis bag.

    Every robot's extended telemetry is routed to its own
    ``/controls_analysis/robot{id}`` topic inside a single output bag.

    :param input_uri: path to the input bag (directory or single file).
    :param output_uri: output bag path; defaults to
        ``<input>_controls_analysis``.
    :param robot_ids: iterable of robot ids to convert; defaults to every robot
        with extended telemetry in the input bag.
    :param use_robot_time: when True (default) stamp output messages (both bag
        timestamp and ``header.stamp``) with each robot's reconstructed robot
        timeline (grounded at each input message's bag time, advanced by
        robot-elapsed time, re-grounded on reboot). When False, the input bag's
        original message timestamp is used.
    :param reboot_reset_threshold_us: backward jump in the robot microsecond
        counter treated as a reboot.
    :param input_storage_id: input storage plugin id (empty = auto-detect).
    :param output_storage_id: output storage plugin id; defaults to the input
        bag's storage id.
    :returns: dict mapping robot_id -> number of messages written.
    """
    if output_uri is None:
        output_uri = default_output_uri(input_uri)
    if robot_ids is None:
        robot_ids = available_robot_ids(input_uri, input_storage_id)
    robot_ids = sorted({int(r) for r in robot_ids})
    if not robot_ids:
        raise ValueError(
            f"No robot extended-telemetry topics found in bag '{input_uri}'.")

    # Map input topic -> robot id, and set up a per-robot output topic + state.
    input_topic_to_id = {default_input_topic(r): r for r in robot_ids}
    output_topics = {r: default_output_topic(r) for r in robot_ids}

    reader = _open_reader(input_uri, input_topic_to_id.keys(), input_storage_id)

    available = {t.name: t.type for t in reader.get_all_topics_and_types()}
    missing = [t for t in input_topic_to_id if t not in available]
    if missing:
        raise ValueError(
            f'Input topics {sorted(missing)} not found in bag '
            f"'{input_uri}'. Available topics: {sorted(available)}")

    if output_storage_id is None:
        output_storage_id = reader.get_metadata().storage_identifier

    writer = _open_writer(
        output_uri, [output_topics[r] for r in robot_ids], output_storage_id)

    clocks = {r: RobotClock(reset_threshold_us=int(reboot_reset_threshold_us))
              for r in robot_ids}
    prev_modes = {r: None for r in robot_ids}
    written = {r: 0 for r in robot_ids}

    while reader.has_next():
        topic, data, bag_time_ns = reader.read_next()
        robot_id = input_topic_to_id.get(topic)
        if robot_id is None:
            continue
        msg = deserialize_message(data, ExtendedTelemetry)

        clock = clocks[robot_id]
        robot_us = robot_timestamp_us(msg.timestamp_us_hi, msg.timestamp_us_lo)
        stamp_ns, _ = clock.update(robot_us, bag_time_ns)
        out_ns = stamp_ns if use_robot_time else bag_time_ns

        sample = sample_from_extended(msg)
        dims = compute_dimensions(sample, prev_modes[robot_id])
        prev_modes[robot_id] = sample.mode

        out = ControlsAnalysis()
        out.header.stamp = Time(nanoseconds=out_ns).to_msg()
        populate_analysis(out, sample, dims, clock.reboot_count)

        writer.write(output_topics[robot_id], serialize_message(out), out_ns)
        written[robot_id] += 1

    reader.close()
    del writer  # flush / close the output bag
    return written


def _build_arg_parser():
    import argparse
    p = argparse.ArgumentParser(
        prog='controls_analysis_bag_convert',
        description="Convert a bag's ExtendedTelemetry into a single combined "
                    'bag of Foxglove-friendly ControlsAnalysis messages, one '
                    'topic per robot (/controls_analysis/robot{id}). By default '
                    'every robot found in the input bag is converted.')
    p.add_argument('input_bag', help='Path to the input ROS bag.')
    p.add_argument('-o', '--output', dest='output_uri', default=None,
                   help='Output bag path (default <input>_controls_analysis).')
    p.add_argument('--robot-ids', type=int, nargs='+', default=None,
                   dest='robot_ids',
                   help='Robots to convert (default: every robot in the bag).')
    p.add_argument('--use-robot-time', action='store_true', default=True,
                   dest='use_robot_time',
                   help='Stamp output with reconstructed robot time (default).')
    p.add_argument('--no-use-robot-time', action='store_false',
                   dest='use_robot_time',
                   help="Keep the input bag's original message timestamps.")
    p.add_argument('--reboot-reset-threshold-us', type=int,
                   default=DEFAULT_REBOOT_RESET_THRESHOLD_US,
                   help='Backward robot-clock jump (us) treated as a reboot.')
    p.add_argument('--input-storage-id', default='',
                   help='Input storage plugin id (empty = auto-detect).')
    p.add_argument('--output-storage-id', default=None,
                   help='Output storage plugin id (default = input storage id).')
    return p


def main(argv=None):
    ns = _build_arg_parser().parse_args(argv)

    if ns.robot_ids is not None:
        robot_ids = sorted({int(r) for r in ns.robot_ids})
    else:
        robot_ids = available_robot_ids(ns.input_bag, ns.input_storage_id)
        if not robot_ids:
            raise SystemExit(
                'No robot extended-telemetry topics '
                "('/robot_feedback/extended/robot{id}') found in bag "
                f"'{ns.input_bag}'.")

    output_uri = ns.output_uri or default_output_uri(ns.input_bag)
    if os.path.exists(output_uri):
        raise SystemExit(
            f"Output bag '{output_uri}' already exists; remove it or pass "
            f'-o/--output.')

    written = convert_bag(
        input_uri=ns.input_bag,
        output_uri=output_uri,
        robot_ids=robot_ids,
        use_robot_time=ns.use_robot_time,
        reboot_reset_threshold_us=ns.reboot_reset_threshold_us,
        input_storage_id=ns.input_storage_id,
        output_storage_id=ns.output_storage_id,
    )
    total = sum(written.values())
    per_robot = ', '.join(f'robot{r}={n}' for r, n in sorted(written.items()))
    print(f"Wrote {total} ControlsAnalysis messages to '{output_uri}' "
          f'({per_robot}).')


if __name__ == '__main__':
    main()
