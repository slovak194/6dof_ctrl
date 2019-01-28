from functools import partial
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph.opengl as gl

import sympy as sp
from sympy.abc import x, y, z, f

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# sp.init_printing(use_unicode=True)

n1, n2, q1, q2 = sp.symbols('n1, n2, q1, q2', integer=False)
x_w, y_w, z_w = sp.symbols('x_w, y_w, z_w', integer=False)
x_e, y_e, z_e = sp.symbols('x_e, y_e, z_e', integer=False)
x_c, y_c, z_c = sp.symbols('x_c, y_c, z_c', integer=False)

n = sp.Matrix([0, 0, -1])
P_w = sp.Matrix([x, y, z])
P_0 = sp.Matrix([0, 0, 0])
P_e = sp.Matrix([x_e, y_e, z_e])

consts = sp.Matrix([n1, n2, f, z_e])
nn1 = 1
# nn2 = 1.33
nn2 = 2.42
ff = 0.01
z_e = 0.1
consts_numeric = [nn1, nn2, ff, z_e]

s1 = P_0 - P_e
s2 = P_e - P_w

x_e, y_e = [sp.solve(n1 * s1.cross(n) - n2 * s2.cross(n), P_e)[key] for key in [x_e, y_e]]

w2e_expr = sp.Matrix([x_e, y_e, z_e])
w2e_lam_full = sp.lambdify((consts, P_w), w2e_expr, 'numpy')
w2e = partial(w2e_lam_full, consts_numeric)

w2c_expr = sp.Matrix([-f * x_e / z_e, -f * y_e / z_e, -f])
w2c_expr.jacobian(P_w)
w2c_lam_full = sp.lambdify((consts, P_w), w2c_expr, 'numpy')
w2c = partial(w2c_lam_full, consts_numeric)

Z = z_e + 10

top_side = np.concatenate((
    np.arange(-10, 11, 1).reshape(1, 21),
    np.full((1, 21), 10),
    np.full((1, 21), Z)
), axis=0)

left_side = np.concatenate((
    np.full((1, 21), 10),
    np.arange(10, -11, -1).reshape(1, 21),
    np.full((1, 21), Z)
), axis=0)

corner_w = np.concatenate((top_side, left_side), axis=1)

corner_e = np.squeeze(np.apply_along_axis(w2e, 0, corner_w))
corner_c = np.squeeze(np.apply_along_axis(w2c, 0, corner_w))

# fig1 = plt.figure()
# ax1 = fig1.add_subplot(111)
#
# plt.axis('equal')
#
# fig2 = plt.figure()
# ax2 = fig2.add_subplot(111, projection='3d')
#
# plt.axis('equal')
#
# ax1.plot(corner_c[0, :], corner_c[1, :], ".")
# ax1.grid()
# ax1.set_aspect('equal')
#
# ax2.plot3D(corner_w[0, :], corner_w[1, :], corner_w[2, :], ".")
# ax2.plot3D(corner_e[0, :], corner_e[1, :], corner_e[2, :], ".")
# ax2.plot3D(corner_c[0, :], corner_c[1, :], corner_c[2, :], ".")
#
# ax2.plot3D([corner_w[0, 0], corner_e[0, 0], corner_c[0, 0]],
#            [corner_w[1, 0], corner_e[1, 0], corner_c[1, 0]],
#            [corner_w[2, 0], corner_e[2, 0], corner_c[2, 0]])
#
# ax2.plot3D([corner_w[0, -1], corner_e[0, -1], corner_c[0, -1]],
#            [corner_w[1, -1], corner_e[1, -1], corner_c[1, -1]],
#            [corner_w[2, -1], corner_e[2, -1], corner_c[2, -1]])
#
# ax2.plot3D([corner_w[0, 10], corner_e[0, 10], corner_c[0, 10]],
#            [corner_w[1, 10], corner_e[1, 10], corner_c[1, 10]],
#            [corner_w[2, 10], corner_e[2, 10], corner_c[2, 10]])
#
# ax2.plot3D([0], [0], [0], "*")
#
# ax2.grid()
# ax2.set_aspect('equal')
#
# plt.show()

app = QtGui.QApplication([])
w = gl.GLViewWidget()
w.show()
g = gl.GLGridItem()
w.addItem(g)

pos = np.random.randint(-10, 10, size=(1000, 3))
pos[:, 2] = np.abs(pos[:, 2])

# sp2 = gl.GLScatterPlotItem(pos=corner_w.T)
w.addItem(gl.GLScatterPlotItem(pos=np.concatenate((corner_w.T, corner_e.T, corner_c.T))))

QtGui.QApplication.instance().exec_()
