from pprint import pprint
import sophus

from pyxacro import get_thrusters_poses
import sympy as sp
import numpy as np


import modern_robotics as mr

th = get_thrusters_poses("../bluerov_ffg/urdf/brov2.xacro")

for k in th.keys():
    th[k]["T_t_b"] = np.linalg.inv(th[k]["T_b_t"])
    th[k]["f_t"] = np.array([[30, 0, 0]]).T  # Assume x alligned
    th[k]["m_t"] = np.array([[0, 0, 0]]).T
    F_t = np.concatenate((th[k]["m_t"], th[k]["f_t"]))
    th[k]["F_t"] = F_t.reshape((6, 1))

for k in th.keys():
    th[k]["f_b"] = th[k]["T_b_t"][0:3, 0:3] @ th[k]["f_t"]
    th[k]["m_b"] = np.cross(th[k]["T_b_t"][0:3, 3:], th[k]["f_b"], axis=0)
    F_b = np.concatenate((th[k]["m_b"], th[k]["f_b"]))
    th[k]["F_b"] = F_b

    th[k]["F_b1"] = mr.Adjoint(th[k]["T_t_b"]).T @ th[k]["F_t"]

    assert(all(th[k]["F_b"] - th[k]["F_b1"] < 10.0**-6))

for k, v in th["thr6"].items():
    print(k)
    print(v)
    print(" ")

exit(0)

# Have effort for all the thrusters
# Create wrenches for all thr
# Express wrenches in s frame
# Summ all wrenches in s frame
# Do forward synamics


# N = np.matrix('1, 2; 4, 4')


# T = sympy.Matrix([[1, 0,  0, 0],
#
#                   [0, 0, -1, 0],
#                   [0, 1,  0, 3],
#                   [0, 0,  0, 1]])
#
#
# se3 = sophus.Se3(sophus.So3(T[0:3, 0:3]), T[0:3, 3])
#
# pprint(se3.Adj())
#
from pprint import pprint
import sys
pprint(sys.path)

