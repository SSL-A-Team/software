#!/usr/bin/env python3

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

r"""
Generate a PlotJuggler layout for the controls analysis topic.

Emits an XML layout with three tabs (X, Y, Theta); each tab vertically stacks
three time-series plots (position / velocity / acceleration) that share a time
axis. Reference-trajectory curves are split per body control mode so each mode
is drawn in its own color with natural gaps at transitions.

Example usage::

  ./scripts/generate_layout.py --robot 0 \\
      --output plotjuggler/controls_analysis_robot0.xml
"""

import argparse
from xml.dom import minidom
from xml.etree import ElementTree as ET

DIMS = ('x', 'y', 'theta')

# Per-mode trajectory colors (kept consistent across every plot/tab).
MODE_COLORS = {
    'global_pos': '#1f77b4',
    'global_vel': '#ff7f0e',
    'local_vel': '#2ca02c',
    'heading_pivot': '#d62728',
    'point_pivot': '#9467bd',
    'heading_line': '#8c564b',
    'point_line': '#e377c2',
}
TRAJ_MODES = tuple(MODE_COLORS.keys())

C_ESTIMATE = '#17becf'
C_VISION = '#7f7f7f'
C_GYRO = '#bcbd22'
C_IMU = '#8c564b'
C_CMD = '#000000'
C_ACCEL_U = '#2ca02c'
C_ACCEL_U_FC = '#d62728'
C_REBOOT = '#ff1493'


def _curves(topic, dim, entries):
    """entries: list of (field, color). Returns list of (name, color)."""
    return [(f'{topic}/{dim}/{field}', color) for field, color in entries]


def _plot_curves(dim, deriv):
    """Return [(field, color)] for the plot of (dim, deriv)."""
    out = []
    if deriv in ('pos', 'vel'):
        out.append((f'{deriv}_estimate', C_ESTIMATE))
        # Color-by-mode reference trajectory (also satisfies "show traj").
        for mode in TRAJ_MODES:
            out.append((f'{deriv}_traj_{mode}', MODE_COLORS[mode]))
    if deriv == 'pos':
        out.append(('pos_vision', C_VISION))
        out.append(('pos_cmd', C_CMD))
    elif deriv == 'vel':
        if dim == 'theta':
            out.append(('vel_gyro', C_GYRO))
        out.append(('vel_cmd', C_CMD))
    elif deriv == 'accel':
        out.append(('accel_u', C_ACCEL_U))
        out.append(('accel_u_fric_comp', C_ACCEL_U_FC))
        if dim in ('x', 'y'):
            out.append(('accel_imu', C_IMU))
        out.append(('accel_cmd', C_CMD))
    return out


def _make_plot(parent, topic, dim, deriv, title):
    area = ET.SubElement(parent, 'DockArea', name=title)
    plot = ET.SubElement(area, 'plot', style='Lines', mode='TimeSeries',
                         flip_y='false', flip_x='false')
    ET.SubElement(plot, 'range', bottom='-1', top='1', left='0', right='1')
    ET.SubElement(plot, 'limitY')
    for name, color in _curves(topic, dim, _plot_curves(dim, deriv)):
        ET.SubElement(plot, 'curve', name=name, color=color)
    # Reboot marker (pulses to 1.0 at each detected reboot) on every plot, so
    # reboots are visible across all stacked views on the shared time axis.
    ET.SubElement(plot, 'curve', name=f'{topic}/reboot_event', color=C_REBOOT)


def build(robot_id):
    topic = f'/controls_analysis/robot{robot_id}'
    root = ET.Element('root')
    tabbed = ET.SubElement(root, 'tabbed_widget', parent='main_window',
                           name='Main Window')
    for dim, label in (('x', 'X'), ('y', 'Y'), ('theta', 'Theta')):
        tab = ET.SubElement(tabbed, 'Tab', containers='1', tab_name=label)
        container = ET.SubElement(tab, 'Container')
        splitter = ET.SubElement(container, 'DockSplitter', count='3',
                                 orientation='-', sizes='0.34;0.33;0.33')
        _make_plot(splitter, topic, dim, 'pos', f'{label} position')
        _make_plot(splitter, topic, dim, 'vel', f'{label} velocity')
        _make_plot(splitter, topic, dim, 'accel', f'{label} acceleration')
    ET.SubElement(tabbed, 'currentTabIndex', index='0')

    ET.SubElement(root, 'use_relative_time_offset', enabled='1')

    plugins = ET.SubElement(root, 'Plugins')
    sub = ET.SubElement(plugins, 'plugin', ID='ROS2 Topic Subscriber')
    topics = ET.SubElement(sub, 'selected_topics')
    ET.SubElement(topics, 'topic', name=topic)

    ET.SubElement(root, 'previouslyLoaded_Datafiles')
    ET.SubElement(root, 'customMathEquations')
    ET.SubElement(root, 'snippets')
    return root


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--robot', type=int, default=0)
    ap.add_argument('--output', type=str, default='')
    args = ap.parse_args()

    root = build(args.robot)
    pretty = minidom.parseString(ET.tostring(root)).toprettyxml(indent=' ')
    if args.output:
        with open(args.output, 'w') as f:
            f.write(pretty)
        print(f'wrote {args.output}')
    else:
        print(pretty)


if __name__ == '__main__':
    main()
