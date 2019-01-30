from pprint import pprint
import sophus

from pyxacro import get_thrusters_poses
import sympy as sp
import numpy as np

# import modern_robotics as mr

th = get_thrusters_poses("../bluerov_ffg/urdf/brov2.xacro")

F_s_summ = sp.Matrix.zeros(6, 1)


for k in th.keys():
    th[k]["T_s_b"] = sp.Matrix(th[k]["T_s_b"])

    # R = th[k]["T_s_b"][0:3, 0:3]
    # p = th[k]["T_s_b"][0:3, 3:]

    R = sp.Matrix(sp.symarray(k + '_R_sb', (3, 3)))
    p = sp.Matrix(sp.symarray(k + '_p_sb', (3, 1)))

    so3_sb = sophus.So3(R)
    se3_sb = sophus.Se3(so3_sb, p)
    se3_bs = se3_sb.inverse()

    F_b = sp.Matrix([sp.Matrix([0, 0, 0]), sp.Matrix([sp.symbols(k + '_fx_b'), 0, 0])])
    F_s = sophus.Se3.Adj(se3_bs).T * F_b

    F_s_summ += F_s

exit(0)

# Have effort for all the thrusters
# Create wrenches for all thr
# Express wrenches in s frame
# Summ all wrenches in s frame
# Do forward synamics

