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
        (0.0055557, 0.0083147),
        (0.0222228, 0.0332588),
        (0.0500013, 0.0748323),
        (0.0888912, 0.133035),
        (0.138893, 0.207867),
        (0.200005, 0.299329),
        (0.272229, 0.40742),
        (0.355565, 0.531139),
        (0.450012, 0.6477),
        (0.552643, 0.74763),
        (0.646794, 0.830932),
        (0.729834, 0.897604),
        (0.801763, 0.947646),
        (0.86258, 0.98106),
        (0.912286, 0.997843),
        (0.950881, 1),
        (0.978364, 1),
        (0.994736, 1),
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
