from pprint import pprint
# import sophus

from pyxacro import get_thrusters_poses
import sympy
import numpy as np


import modern_robotics as mr

th = get_thrusters_poses("../bluerov_ffg/urdf/brov2.xacro")

for k in th.keys():
    th[k]["T_b_s"] = np.linalg.inv(th[k]["T_s_b"])
    th[k]["f_b"] = np.array([[30, 0, 0]]).T  # Assume x alligned
    th[k]["m_b"] = np.array([[0, 0, 0]]).T
    F_b = np.concatenate((th[k]["m_b"], th[k]["f_b"]))
    th[k]["F_b"] = F_b.reshape((6, 1))

for k in th.keys():
    th[k]["f_s"] = th[k]["T_s_b"][0:3, 0:3] @ th[k]["f_b"]
    th[k]["m_s"] = np.cross(th[k]["T_s_b"][0:3, 3:], th[k]["f_s"], axis=0)
    F_s = np.concatenate((th[k]["m_s"], th[k]["f_s"]))
    th[k]["F_s"] = F_s

    th[k]["F_s1"] = mr.Adjoint(th[k]["T_b_s"]).T @ th[k]["F_b"]

    assert(all(th[k]["F_s"] - th[k]["F_s1"] < 10.0**-6))

for k, v in th["thr6"].items():
    print(k)
    print(v)
    print(" ")


exit(0)

# 1. Have effort for all the thrusters
# 2. Create wrenches for all thr
# 3. Express wrenches in s frame
# 4. Summ all wrenches in s frame
# 5. Do forward synamics


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
