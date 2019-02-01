from pprint import pprint
import sophus

from pyxacro import get_robot
import sympy as sp
import numpy as np


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


brov2 = get_robot("../bluerov_ffg/urdf/brov2.urdf")

F_b_summ = sp.Matrix.zeros(6, 1)
ftz = sp.Matrix(sp.symarray('ftz', len(brov2["robot"]["joint"])))  # Along which axis???

for n, thruster in enumerate(brov2["robot"]["joint"]):
    Tbt = sp.Matrix(thruster["Tbt"])

    R = Tbt[0:3, 0:3]
    p = Tbt[0:3, 3:]

    # R = sp.Matrix(sp.symarray(k + '_Rbt', (3, 3)))
    # p = sp.Matrix(sp.symarray(k + '_pbt', (3, 1)))

    so3_bt = sophus.So3(R)
    se3_bt = sophus.Se3(so3_bt, p)

    F_t = sp.Matrix([sp.Matrix([0, 0, 0]), sp.Matrix([0, 0, ftz[n, 0]])])
    F_b = sophus.Se3.Adj(se3_bt.inverse()).T * F_t

    F_b_summ += F_b

# Solve force for each thruster from desired forces and moments in body frame
F_b_desired = sp.Matrix([sp.Matrix(sp.symarray('mb', 3)), sp.Matrix(sp.symarray('fb', 3))])
ftz_from_F_b_desired_expr = F_b_summ - F_b_desired
ftz_from_F_b_desired_result = sp.Matrix(sp.linsolve(ftz_from_F_b_desired_expr.values(), ftz.values()).args[0])

rprint(ftz_from_F_b_desired_expr)
rprint(ftz_from_F_b_desired_result)
rprint(ftz_from_F_b_desired_result.jacobian(F_b_desired))

get_ftz_from_F_b_desired = sp.lambdify((F_b_desired,), ftz_from_F_b_desired_result, 'numpy')
print(get_ftz_from_F_b_desired(np.array([0, 0, 1, 0, 0, 0])))


# Solve force for each thruster from desired angular and linear acceleration in body frame

# m = sp.symbols('m')
# Ixx_b, Iyy_b, Izz_b = sp.symbols("Ixx_b Iyy_b Izz_b")
# Ixy_b, Ixz_b, Iyz_b = sp.symbols("Ixy_b Ixz_b Iyz_b")

m = brov2["robot"]["link"][0]["inertial"]["mass"]["@value"]
I_b = brov2["robot"]["link"][0]["inertial"]["inertia"]
Ixx_b = I_b["@ixx"]
Iyy_b = I_b["@iyy"]
Izz_b = I_b["@izz"]
Ixy_b = I_b["@ixy"]
Ixz_b = I_b["@ixz"]
Iyz_b = I_b["@iyz"]

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


ftz_from_V_b_dV_b_expr = G_b * dV_b - AdV_b.T * G_b * V_b - F_b_summ

ftz_from_V_b_dV_b_result = sp.Matrix([sp.solve(ftz_from_V_b_dV_b_expr, ftz)[key] for key in ftz])

rprint(ftz_from_V_b_dV_b_expr)
rprint(ftz_from_V_b_dV_b_result)
rprint(ftz_from_V_b_dV_b_result.jacobian(dV_b))

get_ftz_from_V_b_dV_b = sp.lambdify((V_b, dV_b), ftz_from_V_b_dV_b_result, 'numpy')

print(get_ftz_from_V_b_dV_b(np.array([0, 0, 0, 0, 0, 0]), np.array([0, 0, 0.1, 0, 0, 0])))


####

# M_i = -h_i*w_i - k_i*qw*qv_i  -->
#
# k_i = I_i
# h_i = I_i
