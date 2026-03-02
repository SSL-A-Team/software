"""Docstring for ateam_software.motion.ateam_path_planning.visualize_paths."""

import matplotlib.pyplot as plt
import matplotlib.patches as patches

paths = [
  [
    (0, 0),
    (0.005, 0),
    (0.02, 0),
  ],
  [
    (0.045, 0),
    (0.0779513, 0.00403606),
    (0.116805, 0.0161442),
    (0.161562, 0.0363245),
    (0.212221, 0.0645769),
    (0.268763, 0.100901),
    (0.327789, 0.145298),
    (0.386815, 0.197767),
    (0.445842, 0.258308),
    (0.504868, 0.32692),
    (0.563894, 0.403606),
    (0.622921, 0.484327),
    (0.681947, 0.565048),
    (0.739869, 0.644259),
    (0.792333, 0.716006),
    (0.838895, 0.779682),
    (0.879555, 0.835285),
    (0.914311, 0.882817),
    (0.943165, 0.922276),
    (0.966117, 0.953663),
    (0.983165, 0.976978),
    (0.994311, 0.992221),
    (0.999555, 0.999391),
  ],
]

obstacles = [
  (0.5, 0.5, 0.1)
]


if __name__ == "__main__":
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
      circ = patches.Circle((x, y), radius=radius, fill=False, edgecolor=color)
      ax.add_patch(circ)

  for (ox, oy, orad) in obstacles:
    obs = patches.Circle((ox, oy), radius=orad, fill=True, color='gray', alpha=0.5)
    ax.add_patch(obs)

  ax.set_aspect('equal', 'box')
  ax.autoscale_view()
  plt.show()


