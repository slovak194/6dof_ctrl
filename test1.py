from pprint import pprint
import sophus

from pyxacro import get_thrusters_poses


import sympy
import numpy as np

th = get_thrusters_poses("../../bluerov_ffg/urdf/brov2.xacro")

for k in th.keys():
    th[k]["f_b"] = np.array([30, 0, 0], ndmin=1)  # Assume x alligned

for k in th.keys():
    th[k]["f_s"] = np.matmul(th[k]["T_s_b"][0:3, 0:3], th[k]["f_b"])
    th[k]["m_s"] = np.cross(th[k]["T_s_b"][0:3, 3], th[k]["f_s"])
    F_s = np.concatenate((th[k]["m_s"], th[k]["f_s"]))
    th[k]["F_s"] = F_s.reshape((6, 1))

pprint(th)


T = sympy.Matrix([[1, 0,  0, 0],
                  [0, 0, -1, 0],
                  [0, 1,  0, 3],
                  [0, 0,  0, 1]])

se3 = sophus.Se3(sophus.So3(T[0:3, 0:3]), T[0:3, 3])

pprint(se3.Adj())

