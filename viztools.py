import matplotlib.pyplot as plt
import matplotlib.animation as anim
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d import proj3d
from matplotlib.patches import FancyArrowPatch
import numpy as np

from pyxacro import get_thrusters_poses


class Arrow3D(FancyArrowPatch):
    """Custom class for 3D arrows"""

    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0, 0), (0, 0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        FancyArrowPatch.draw(self, renderer)

    def set_data(self, xs, ys, zs):
        self._verts3d = xs, ys, zs


th = get_thrusters_poses("../bluerov_ffg/urdf/brov2.xacro")

fig = plt.figure()
ax = Axes3D(fig)

artists = {}

# keys = ["thr4"]
keys = th.keys()

for k in keys:
    artists[k] = []

    Tbt = th[k]["Tbt"]

    artists[k].append(ax.scatter(Tbt[0, 3:][0], Tbt[1, 3:][0], Tbt[2, 3:][0], s=10, c='black'))

    mutation_scale = 0.1
    sc = 0.05

    for axis, color in zip([0, 1, 2], ["red", "green", "blue"]):
        artists[k].append(Arrow3D(
            [Tbt[0, 3:][0], Tbt[0, 3:][0] + Tbt[0, axis] * sc],
            [Tbt[1, 3:][0], Tbt[1, 3:][0] + Tbt[1, axis] * sc],
            [Tbt[2, 3:][0], Tbt[2, 3:][0] + Tbt[2, axis] * sc],
            arrowstyle="-|>", lw=2, mutation_scale=mutation_scale, color=color))

    for artist in artists[k]:
        ax.add_artist(artist)


ax.set_aspect('equal')

box = 0.2

ax.set_xlim(-box, box)
ax.set_ylim(-box, box)
ax.set_zlim(-box, box)

plt.show()
