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

"""Docstring for ateam_software.motion.ateam_path_planning.visualize_paths."""

import matplotlib.patches as patches
import matplotlib.pyplot as plt

paths = [
    [
        (0, 0),
        (0.0147118, 0),
        (0.0588471, 0),
    ],
    [
        (0.132406, 0),
        (0.22901, 0.012472),
        (0.342282, 0.0498882),
        (0.471754, 0.112248),
        (0.596118, 0.199553),
        (0.703815, 0.311801),
        (0.794845, 0.448994),
        (0.869208, 0.599987),
        (0.926903, 0.728781),
        (0.967932, 0.83263),
        (0.992293, 0.911535),
        (1, 0.965496),
        (1, 0.994513),
    ],
]

obstacles = [
    (0.5, 0.5, 0.1)
]


if __name__ == '__main__':
    fig, ax = plt.subplots()
    diameter = 0.18
    radius = diameter / 2.0
    cmap = plt.get_cmap('tab10')

    for i, path in enumerate(paths):
        xs = [p[0] for p in path]
        ys = [p[1] for p in path]
        color = cmap(i % 10)
        ax.plot(xs, ys, '-', color=color)
        for (x, y) in path:
            circ = patches.Circle((x, y), radius=radius,
                                  fill=False, edgecolor=color)
            ax.add_patch(circ)

    for (ox, oy, orad) in obstacles:
        obs = patches.Circle((ox, oy), radius=orad,
                             fill=True, color='gray', alpha=0.5)
        ax.add_patch(obs)

    ax.set_aspect('equal', 'box')
    ax.autoscale_view()
    plt.show()
