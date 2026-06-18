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
        (-1, 0.2),
        (-0.991666, 0.212472),
        (-0.966666, 0.249888),
        (-0.924998, 0.312248),
        (-0.862525, 0.378747),
        (-0.775107, 0.428579),
        (-0.662746, 0.461743),
        (-0.52544, 0.478241),
        (-0.36319, 0.478071),
        (-0.175996, 0.461234),
        (0.0358604, 0.42773),
        (0.242704, 0.377559),
        (0.424603, 0.313519),
        (0.581558, 0.260338),
        (0.713569, 0.223823),
        (0.820636, 0.203977),
        (0.902758, 0.2),
        (0.959937, 0.2),
        (0.992172, 0.2),
    ],
    [
        (-1, -0.2),
        (-0.991666, -0.212472),
        (-0.966666, -0.249888),
        (-0.924998, -0.312248),
        (-0.862525, -0.378747),
        (-0.775107, -0.428579),
        (-0.662746, -0.461743),
        (-0.52544, -0.478241),
        (-0.36319, -0.478071),
        (-0.175996, -0.461234),
        (0.0358604, -0.42773),
        (0.242704, -0.377559),
        (0.424603, -0.313519),
        (0.581558, -0.260338),
        (0.713569, -0.223823),
        (0.820636, -0.203977),
        (0.902758, -0.2),
        (0.959937, -0.2),
        (0.992172, -0.2),
    ],
]

obstacles = [
    (0.0, 0.0, 0.1)
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
