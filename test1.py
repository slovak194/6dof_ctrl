from pprint import pprint
import sophus

import sympy
import numpy as np

import modern_robotics as mb

th = {}

thr12_x = 0.125
thr12_y = 0.115
thr34_x = 0.15
thr34_y = 0.11
thr_z = -0.07
thr12_theta = 0.7
thr34_theta = 0.6
thr56_y = 0.11

th["thr1"] = {}
th["thr1"]["xyz"] = np.array([thr12_x, -thr12_y, thr_z])
th["thr1"]["rpy"] = np.array([0.0, np.pi/2, thr12_theta])
th["thr2"] = {}
th["thr2"]["xyz"] = np.array([thr12_x, thr12_y, thr_z])
th["thr2"]["rpy"] = np.array([0.0, np.pi/2, -thr12_theta])
th["thr3"] = {}
th["thr3"]["xyz"] = np.array([-thr34_x, -thr34_y, thr_z])
th["thr3"]["rpy"] = np.array([0.0, np.pi/2, np.pi-thr34_theta])
th["thr4"] = {}
th["thr4"]["xyz"] = np.array([-thr34_x, thr34_y, thr_z])
th["thr4"]["rpy"] = np.array([0.0, np.pi/2, np.pi+thr34_theta])
th["thr5"] = {}
th["thr5"]["xyz"] = np.array([0.0, -thr56_y, 0])
th["thr5"]["rpy"] = np.array([0.0, 0.0, 0.0])
th["thr6"] = {}
th["thr6"]["xyz"] = np.array([0.0, thr56_y, 0])
th["thr6"]["rpy"] = np.array([0.0, 0.0, 0.0])

T = sympy.Matrix([[1, 0,  0, 0],
                  [0, 0, -1, 0],
                  [0, 1,  0, 3],
                  [0, 0,  0, 1]])

se3 = sophus.Se3(sophus.So3(T[0:3, 0:3]), T[0:3, 3])

pprint(se3.Adj())

# mb.Adjoint()
