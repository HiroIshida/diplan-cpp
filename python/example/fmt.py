import time

import matplotlib.pyplot as plt
import numpy as np

from disbmp import BoundingBox, FastMarchingTree, State

sbound = BoundingBox([0.0, 0.0, -0.2, -0.2], [1.0, 1.0, 0.2, 0.2])
s0 = State([0.1, 0.1, 0.0, 0.0])
s1 = State([0.9, 0.9, 0.0, 0.0])
r = 0.4


def is_obstacle_free(s):
    vec = s.to_vector()
    x = vec[:2]
    return np.linalg.norm(x - 0.5) > r


N = 1000
fmt = FastMarchingTree(s0, s1, is_obstacle_free, sbound, 0.1, 1.0, N)
ts = time.time()
is_solved = fmt.solve(N)
print("Solved: {}, time: {}".format(is_solved, time.time() - ts))
assert is_solved

solution = fmt.get_solution()
motions = fmt.get_all_motions()

fig, ax = plt.subplots()
T = solution.get_duration()
times = np.linspace(0, T, 100)
ps = np.array([solution.interpolate(t) for t in times])
ax.plot(ps[:, 0], ps[:, 1], color="k", linewidth=0.5)

# visualize start and goal
ax.scatter(0.1, 0.1, color="r", s=10)
ax.scatter(0.9, 0.9, color="g", s=10)

# visualize circle
circle = plt.Circle((0.5, 0.5), r, color="k", fill=False)
ax.add_artist(circle)

# visualize boundary
boundary = plt.Rectangle((0.0, 0.0), 1.0, 1.0, color="k", fill=False)
ax.add_artist(boundary)

plt.show()
