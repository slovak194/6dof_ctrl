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

    Tt = np.identity(4)

    Tb = th[k]["Tbt"] @ Tt

    artists[k].append(ax.scatter(Tb[0, 3:][0], Tb[1, 3:][0], Tb[2, 3:][0], s=10, c='black'))

    mutation_scale = 0.1
    sc = 0.05

    axis = 0
    artists[k].append(Arrow3D(
        [Tb[0, 3:][0], Tb[0, 3:][0] + Tb[0, axis] * sc],
        [Tb[1, 3:][0], Tb[1, 3:][0] + Tb[1, axis] * sc],
        [Tb[2, 3:][0], Tb[2, 3:][0] + Tb[2, axis] * sc],
        arrowstyle="-|>", lw=2, mutation_scale=mutation_scale, color="red"))

    axis = 1
    artists[k].append(Arrow3D(
        [Tb[0, 3:][0], Tb[0, 3:][0] + Tb[0, axis] * sc],
        [Tb[1, 3:][0], Tb[1, 3:][0] + Tb[1, axis] * sc],
        [Tb[2, 3:][0], Tb[2, 3:][0] + Tb[2, axis] * sc],
        arrowstyle="-|>", lw=2, mutation_scale=mutation_scale, color="green"))

    axis = 2
    artists[k].append(Arrow3D(
        [Tb[0, 3:][0], Tb[0, 3:][0] + Tb[0, axis] * sc],
        [Tb[1, 3:][0], Tb[1, 3:][0] + Tb[1, axis] * sc],
        [Tb[2, 3:][0], Tb[2, 3:][0] + Tb[2, axis] * sc],
        arrowstyle="-|>", lw=2, mutation_scale=mutation_scale, color="blue"))

    for artist in artists[k]:
        ax.add_artist(artist)


ax.set_aspect('equal')

box = 1

ax.set_xlim(-box, box)
ax.set_ylim(-box, box)
ax.set_zlim(-box, box)

plt.show()
