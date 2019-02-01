from pprint import pprint
import sophus

from pyxacro import get_thrusters_poses
import sympy as sp
import numpy as np

import modern_robotics as mr

def round_expr(expr, num_digits):
    return expr.xreplace({n : round(n, num_digits) for n in expr.atoms(sp.Number)})


def rprint(expr):
    pprint(round_expr(expr, 3))


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
# fz = sp.Matrix(sp.symarray('fz', len(th.keys())))  # Along which axis???
fz_0, fz_1, fz_2, fz_3, fz_4, fz_5 = sp.symbols('fz_0, fz_1, fz_2, fz_3, fz_4, fz_5')
fz = sp.Matrix([fz_0, fz_1, fz_2, fz_3, fz_4, fz_5])

# fz = sp.MatrixSymbol('fz', 6, 1)

for n, k in enumerate(th.keys()):
    th[k]["Tbt"] = sp.Matrix(th[k]["Tbt"])

    rprint(th[k]["Tbt"])

    R = th[k]["Tbt"][0:3, 0:3]
    p = th[k]["Tbt"][0:3, 3:]

    # R = sp.Matrix(sp.symarray(k + '_Rbt', (3, 3)))
    # p = sp.Matrix(sp.symarray(k + '_pbt', (3, 1)))

    so3_bt = sophus.So3(R)
    se3_bt = sophus.Se3(so3_bt, p)

    F_t = sp.Matrix([sp.Matrix([0, 0, 0]), sp.Matrix([0, 0, fz[n, 0]])])
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

G_b = sp.Matrix.zeros(6, 6)
G_b[0:3, 0:3] = I_b
G_b[3:, 3:] = m * sp.Matrix.eye(3)

# Feneral case:

V_b = sp.Matrix([sp.Matrix(sp.symarray('w', 3)), sp.Matrix(sp.symarray('v', 3))])
dV_b = sp.Matrix([sp.Matrix(sp.symarray('dw', 3)), sp.Matrix(sp.symarray('dv', 3))])
AdV_b = adv(V_b)

expr = G_b * dV_b - AdV_b.T * G_b * V_b - F_b_summ

fz_res_dict = sp.solve(expr, fz)   # Along which axis??? fx or fz?
fz_res = sp.Matrix([fz_res_dict[key] for key in fz])

rprint(expr)
rprint(fz_res)

fz_res.jacobian(dV_b)

fz_res_num = sp.lambdify((V_b, dV_b), fz_res, 'numpy')

V_b_numeric = np.array([0, 0, 0, 0, 0, 0])
dV_b_numeric = np.array([0, 0, 0.1, 0, 0, 0])
print(fz_res_num(V_b_numeric, dV_b_numeric))


####

# F_b_summ.applyfunc(lambda ij: ij.coeff(fz_2))


# M_i = -h_i*w_i - k_i*qw*qv_i  -->
#
# k_i = I_i
# h_i = I_i

