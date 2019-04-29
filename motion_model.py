from pprint import pprint
import sophus

from pyxacro import get_robot
import sympy as sp
import sympy.utilities.codegen as sgen
import numpy as np


def round_expr(expr, num_digits):
    return expr.xreplace({n: round(n, num_digits) for n in expr.atoms(sp.Number)})


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


def get_robot_parameters(urdf_file_path):
    robot = get_robot(urdf_file_path)

    thrusters_mapping = sp.Matrix.zeros(6, 6)
    F_c_summ = sp.Matrix.zeros(6, 1)
    ftz = sp.Matrix(sp.symarray('ftz', len(robot["robot"]["joint"])))  # Along which axis???

    for n, thruster in enumerate(robot["robot"]["joint"]):
        Tct = thruster["Tct"]
        Q, _ = np.linalg.qr(Tct[0:3, 0:3])
        Tct[0:3, 0:3] = Q
        Tct = sp.Matrix(Tct)

        # print(" ")
        # print(thruster["@name"])
        # pprint(Tct)
        # print(" ")

        R = Tct[0:3, 0:3]
        p = Tct[0:3, 3:]

        so3_ct = sophus.So3(R)
        se3_ct = sophus.Se3(so3_ct, p)

        F_t = sp.Matrix([sp.Matrix([0, 0, 0]), sp.Matrix([0, 0, ftz[n, 0]])])
        # (3.95) Express wrench in another basis.
        # MODERN ROBOTICS MECHANICS, PLANNING, AND CONTROL Kevin M. Lynch and Frank C. Park May 3, 2017
        T_adj_tc = sophus.Se3.Adj(se3_ct.inverse()).T
        F_c = T_adj_tc * F_t

        # pprint(F_c)

        F_c_summ += F_c
        thrusters_mapping[:, n] = T_adj_tc * sp.Matrix([sp.Matrix([0, 0, 0]), sp.Matrix([0, 0, 1])])

    m = robot["robot"]["link"][0]["inertial"]["mass"]["@value"]
    I_c = robot["robot"]["link"][0]["inertial"]["inertia"]
    Ixx_c = I_c["@ixx"]
    Iyy_c = I_c["@iyy"]
    Izz_c = I_c["@izz"]
    Ixy_c = I_c["@ixy"]
    Ixz_c = I_c["@ixz"]
    Iyz_c = I_c["@iyz"]

    I_c = sp.Matrix([[Ixx_c, Ixy_c, Ixz_c],
                     [Ixy_c, Iyy_c, Iyz_c],
                     [Ixz_c, Iyz_c, Izz_c]])

    robot["F_c_summ"] = F_c_summ
    robot["thrusters_mapping"] = thrusters_mapping
    robot["ftz"] = ftz
    robot["I_c"] = I_c
    robot["m"] = m

    return robot


robot = get_robot_parameters("../bluerov_ffg/urdf/brov2.urdf")
# robot = get_robot_parameters("../bluerov_ffg/urdf/brov2_original.urdf")


def get_get_ftz_from_F_c():
    # Solve force for each thruster from desired forces and moments in body frame
    mcx, mcy, mcz = sp.symbols("mcx, mcy, mcz")
    # mcy = sp.Symbol('0')
    mc = sp.Matrix([mcx, mcy, mcz])

    fcx, fcy, fcz = sp.symbols("fcx, fcy, fcz")
    fc = sp.Matrix([fcx, fcy, fcz])

    F_c = sp.Matrix([mc, fc])
    ftz_from_F_c_expr = robot["F_c_summ"] - F_c
    ftz_from_F_c_result = sp.Matrix(sp.linsolve(ftz_from_F_c_expr.values(), robot["ftz"].values()).args[0])


    # A = robot["thrusters_mapping"]
    # ftz_from_F_c_result = (A.T*A).inv()*A.T*F_c  # Least squares


    return {
        "lambda": sp.lambdify((F_c,), ftz_from_F_c_result, 'numpy'),
        "s_lambda": sp.lambdify((F_c,), ftz_from_F_c_result, 'sympy'),
        "result": ftz_from_F_c_result,
        "expr": ftz_from_F_c_expr,
        "J_wrt_F_c": ftz_from_F_c_result.jacobian(F_c)
    }


G_c = sp.Matrix.zeros(6, 6)
G_c[0:3, 0:3] = robot["I_c"]
G_c[3:, 3:] = robot["m"] * sp.Matrix.eye(3)
w_c = sp.Matrix(sp.symarray('w_c', 3))
v_c = sp.Matrix(sp.symarray('v_c', 3))
V_c = sp.Matrix([w_c, v_c])

dV_c = sp.Matrix([sp.Matrix(sp.symarray('dw', 3)), sp.Matrix(sp.symarray('dv', 3))])
AdV_c = adv(V_c)


def get_get_ftz_from_V_c_dV_c():
    # (8.40) MODERN ROBOTICS MECHANICS, PLANNING, AND CONTROL Kevin M. Lynch and Frank C. Park May 3, 2017
    ftz_from_V_c_dV_c_expr = G_c * dV_c - AdV_c.T * G_c * V_c - robot["F_c_summ"]
    ftz_from_V_c_dV_c_result = sp.Matrix([sp.solve(ftz_from_V_c_dV_c_expr, robot["ftz"])[key] for key in robot["ftz"]])

    return {
        "lambda": sp.lambdify((V_c, dV_c), ftz_from_V_c_dV_c_result, 'numpy'),
        "s_lambda": sp.lambdify((V_c, dV_c), ftz_from_V_c_dV_c_result, 'sympy'),
        "result": ftz_from_V_c_dV_c_result,
        "expr": ftz_from_V_c_dV_c_expr,
        "J_wrt_V_c": ftz_from_V_c_dV_c_result.jacobian(V_c),
        "J_wrt_dV_c": ftz_from_V_c_dV_c_result.jacobian(dV_c),
    }

t_sc = sp.Matrix(sp.symarray('t_sc', 3))
q_s_cog = sp.Matrix(sp.symbols('q_sc_w q_sc_x q_sc_y q_sc_z'))  # world to COG
q_sc = sophus.Quaternion(q_s_cog[0], q_s_cog[1:4, :])
T_sc = sophus.Se3(sophus.So3(q_sc), t_sc)

t_st = sp.Matrix(sp.symarray('t_st', 3))
q_s_target = sp.Matrix(sp.symbols('q_st_w q_st_x q_st_y q_st_z'))  # world to target
q_st = sophus.Quaternion(q_s_target[0], q_s_target[1:4, :])
T_st = sophus.Se3(sophus.So3(q_st), t_st)


def get_pose_control():
    w_gain = sp.Matrix(sp.symarray('w_gain', 3))
    q_gain = sp.Matrix(sp.symarray('q_gain', 3))
    v_gain = sp.Matrix(sp.symarray('v_gain', 3))
    t_gain = sp.Matrix(sp.symarray('t_gain', 3))

    ctrl_gains = (w_gain, q_gain, v_gain, t_gain)

    T_ct = (T_st.inverse() * T_sc).inverse()

    m_c = sp.matrix_multiply_elementwise(-w_gain, w_c) + \
          sp.matrix_multiply_elementwise(q_gain, T_ct.so3.q.vec) * T_ct.so3.q.real

    f_c = sp.matrix_multiply_elementwise(-v_gain, v_c) + \
          sp.matrix_multiply_elementwise(t_gain, T_ct.t)

    F_c = m_c.col_join(f_c)
    return {
        "lambda": sp.lambdify((t_sc, q_s_cog, w_c, v_c, t_st, q_s_target, ctrl_gains), F_c, 'numpy'),
        "s_lambda": sp.lambdify((t_sc, q_s_cog, t_st, q_s_target, ctrl_gains), F_c, 'sympy'),
        "result": F_c,
    }


def get_get_dV_c_from_target_T():
    w_gain = sp.Matrix(sp.symarray('w_gain', 3))
    q_gain = sp.Matrix(sp.symarray('q_gain', 3))
    v_gain = sp.Matrix(sp.symarray('v_gain', 3))
    t_gain = sp.Matrix(sp.symarray('t_gain', 3))

    ctrl_gains = (w_gain, q_gain, v_gain, t_gain)

    T_ct = (T_st.inverse() * T_sc).inverse()

    dw_c = sp.matrix_multiply_elementwise(-w_gain, w_c) + \
          sp.matrix_multiply_elementwise(q_gain, T_ct.so3.q.vec) * T_ct.so3.q.real

    dv_c = sp.matrix_multiply_elementwise(-v_gain, v_c) + \
          sp.matrix_multiply_elementwise(t_gain, T_ct.t)

    dV_c_target = dw_c.col_join(dv_c)

    get_ftz_from_V_c_dV_c = get_get_ftz_from_V_c_dV_c()["s_lambda"]

    ftz_from_V_c_dV_c_result = get_ftz_from_V_c_dV_c(V_c, dV_c_target)

    return {
        "lambda": sp.lambdify((t_sc, q_s_cog, w_c, v_c, t_st, q_s_target, ctrl_gains), ftz_from_V_c_dV_c_result, 'numpy'),
        "s_lambda": sp.lambdify((t_sc, q_s_cog, t_st, q_s_target, ctrl_gains), ftz_from_V_c_dV_c_result, 'sympy'),
        "result": ftz_from_V_c_dV_c_result,
    }


if __name__ == '__main__':

    pose_control = get_get_dV_c_from_target_T()

    pprint(pose_control)

    # get_ftz_from_F_c = get_get_ftz_from_F_c()
    # pprint(get_ftz_from_F_c["s_lambda"](sp.Matrix(sp.symarray("F", 6))))

    # pprint(get_ftz_from_F_c["J_wrt_F_c"])
    #
    # get_ftz_from_V_c_dV_c = get_get_ftz_from_V_c_dV_c()
    # print(get_ftz_from_V_c_dV_c["lambda"](np.array([0, 0, 0, 0, 0, 0]), np.array([0, 0, 0.1, 0, 0, 0])))



# [(c_name, c_code), (h_name, c_header)] = \
#     sgen.codegen(("get_ftz_from_F_c", sp.Eq(sp.MatrixSymbol('ftz', 6, 1), ftz_from_F_c_result)),
#                  language="C99",
#                  prefix="get_ftz_from_F_c",
#                  to_files=False,
#                  header=False,
#                  empty=True)
#
#
# print(c_code)
#
# [(m_name, m_code)] = \
#     sgen.codegen(("get_ftz_from_F_c", sp.Eq(sp.MatrixSymbol('ftz', 6, 1), ftz_from_F_c_result)),
#                  language="Octave",
#                  to_files=False,
#                  header=False,
#                  empty=True)
# print(m_name)
# print(m_code)

# Solve force for each thruster from desired angular and linear acceleration in body frame

# m = sp.symbols('m')
# Ixx_b, Iyy_b, Izz_b = sp.symbols("Ixx_b Iyy_b Izz_b")
# Ixy_b, Ixz_b, Iyz_b = sp.symbols("Ixy_b Ixz_b Iyz_b")
