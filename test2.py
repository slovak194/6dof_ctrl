from pprint import pprint
import sophus

from pyxacro import get_thrusters_poses
import sympy as sp
import numpy as np

import modern_robotics as mr


def adv(v):
    a = sophus.So3.hat(v[0:3])
    b = sophus.So3.hat(v[3:6])
    res = sp.Matrix.zeros(6, 6)
    res[0:3, 0:3] = a
    res[3:, 3:] = a
    res[3:, 0:3] = b
    return res


th = get_thrusters_poses("../bluerov_ffg/urdf/brov2.xacro")

F_b_summ = sp.Matrix.zeros(6, 1)

th_f_x = sp.symarray('th_f_x', (len(th.keys()), 1))

for n, k in enumerate(th.keys()):
    th[k]["T_b_t"] = sp.Matrix(th[k]["T_b_t"])

    R = th[k]["T_b_t"][0:3, 0:3]
    p = th[k]["T_b_t"][0:3, 3:]

    # R = sp.Matrix(sp.symarray(k + '_R_b_t', (3, 3)))
    # p = sp.Matrix(sp.symarray(k + '_p_b_t', (3, 1)))

    so3_bt = sophus.So3(R)
    se3_bt = sophus.Se3(so3_bt, p)

    F_t = sp.Matrix([sp.Matrix([0, 0, 0]), sp.Matrix([th_f_x[n, 0], 0, 0])])
    F_b = sophus.Se3.Adj(se3_bt.inverse()).T * F_t

    F_b_summ += F_b


# m = sp.symbols('m')
# Ixx_b, Iyy_b, Izz_b = sp.symbols("Ixx_b Iyy_b Izz_b")
# Ixy_b, Ixz_b, Iyz_b = sp.symbols("Ixy_b Ixz_b Iyz_b")

m = 7.5
Ixx_b = 0.099
Iyy_b = 0.129
Izz_b = 0.16
Ixy_b = 0.0
Ixz_b = 0.0056
Iyz_b = 0

I_b = sp.Matrix([ [Ixx_b, Ixy_b,  Ixz_b],
                  [Ixy_b, Iyy_b,  Iyz_b],
                  [Ixz_b, Iyz_b,  Izz_b]])

V_b = sp.Matrix([sp.Matrix(sp.symarray('w', (3, 1))), sp.Matrix(sp.symarray('v', (3, 1)))])
dV_b = sp.Matrix([sp.Matrix(sp.symarray('dw', (3, 1))), sp.Matrix(sp.symarray('dv', (3, 1)))])

AdV_b = adv(V_b)

G_b = sp.Matrix.zeros(6, 6)
G_b[0:3, 0:3] = I_b
G_b[3:, 3:] = m * sp.Matrix.eye(3)


expr = G_b * dV_b - AdV_b.T * G_b * V_b - F_b_summ


# F_b = G_b * dV_b - [AdV_b].T * G_b * V_b


# Have effort for all the thrusters
# Create wrenches for all thr
# Express wrenches in s frame
# Summ all wrenches in s frame
# Do forward synamics

