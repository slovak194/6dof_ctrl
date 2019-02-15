import pybullet as p
import pybullet_data
import time
from enum import Enum

import numpy as np
from quaternion import from_euler_angles
from quaternion import as_rotation_matrix
from quaternion import as_float_array

import quaternion as nq

from pyxacro import get_robot
from motion_model import get_get_ftz_from_F_c
from motion_model import get_pose_control

get_ftz_from_F_c = get_get_ftz_from_F_c()["lambda"]

physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0, 0, 0)

# p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "brov2.mp4")

planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0, 0, 1]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])

urdf_file_path = "../bluerov_ffg/urdf/brov2.urdf"

brov2 = get_robot(urdf_file_path)

robot_id = p.loadURDF(urdf_file_path, cubeStartPos, cubeStartOrientation, flags=p.URDF_USE_INERTIA_FROM_FILE)

pi = np.pi

l_T_st_6dpf = [
    {"t_st": np.array([-0.5, 0.0, 1.0]), "q_st": from_euler_angles(0, pi/4, 0)},
    {"t_st": np.array([-0.5, 0.5, 1.0]), "q_st": from_euler_angles(-pi/4, pi/4, 0)},
    {"t_st": np.array([-0.5, 0.0, 1.0]), "q_st": from_euler_angles(0, pi/4, 0)},
    {"t_st": np.array([-0.5, -0.5, 1.0]), "q_st": from_euler_angles(pi/4, pi/4, 0)},
]

l_T_st_translation = [
    {"t_st": np.array([1.0, 1.0, 1.0]), "q_st": from_euler_angles(0, 0, 0)},
    {"t_st": np.array([-1.0, 1.0, 1.0]), "q_st": from_euler_angles(0, 0, 0)},
    {"t_st": np.array([-1.0, -1.0, 1.0]), "q_st": from_euler_angles(0, 0, 0)},
    {"t_st": np.array([1.0, -1.0, 1.0]), "q_st": from_euler_angles(0, 0, 0)},
]

l_T_st_rotation = [
    {"t_st": np.array([0.0, 0.0, 1.0]), "q_st": from_euler_angles(pi/4, pi/3, 0)},
    {"t_st": np.array([0.0, 0.0, 1.0]), "q_st": from_euler_angles(-pi/4, pi/3, 0)},
    {"t_st": np.array([0.0, 0.0, 1.0]), "q_st": from_euler_angles(-pi/4, -pi/3, 0)},
    {"t_st": np.array([0.0, 0.0, 1.0]), "q_st": from_euler_angles(pi/4, -pi/3, 0)},
]

q_gain = np.array([
    brov2["robot"]["link"][0]["inertial"]["inertia"]["@ixx"],
    brov2["robot"]["link"][0]["inertial"]["inertia"]["@iyy"],
    brov2["robot"]["link"][0]["inertial"]["inertia"]["@izz"]
], dtype=np.float64)

w_gain = np.array([
    brov2["robot"]["link"][0]["inertial"]["inertia"]["@ixx"],
    brov2["robot"]["link"][0]["inertial"]["inertia"]["@iyy"],
    brov2["robot"]["link"][0]["inertial"]["inertia"]["@izz"]
], dtype=np.float64)

l_T_st = l_T_st_6dpf
# l_T_st = l_T_st_translation
# l_T_st = l_T_st_rotation

w_gain = w_gain * 10
q_gain = q_gain * 10

v_gain = np.array([1.0, 1.0, 1.0]) * 10
t_gain = np.array([1.0, 1.0, 1.0]) * 5

ctrl_gains = (w_gain, q_gain, v_gain, t_gain)



db_graph = {}
# dbgraph["x"] = p.addUserDebugLine([0, 0, 0], [1, 0, 0], [1, 0, 0], parentObjectUniqueId=robot_id, parentLinkIndex=-1)
# dbgraph["y"] = p.addUserDebugLine([0, 0, 0], [0, 1, 0], [0, 1, 0], parentObjectUniqueId=robot_id, parentLinkIndex=-1)
# dbgraph["z"] = p.addUserDebugLine([0, 0, 0], [0, 0, 1], [0, 0, 1], parentObjectUniqueId=robot_id, parentLinkIndex=-1)

db_graph["w_c"] = p.addUserDebugLine([0, 0, 0], [0, 0, 1], [1, 0, 0], parentObjectUniqueId=robot_id,
                                           parentLinkIndex=-1)
db_graph["ftz"] = []

for link_idx in range(0, 6):
    db_graph["ftz"].append(p.addUserDebugLine([0, 0, 0], [0, 0, 1], [0, 1, 0],
                                              parentObjectUniqueId=robot_id,
                                              parentLinkIndex=link_idx))


# s - origin
# t - target
# c - center of gravity

def get_state():
    s = {}
    t_sc, q_sc = p.getBasePositionAndOrientation(robot_id)
    t_sc = np.array(t_sc, ndmin=2).T
    q_sc = nq.quaternion(q_sc[3], q_sc[0], q_sc[1], q_sc[2])
    R_sc = as_rotation_matrix(q_sc)
    R_cs = as_rotation_matrix(q_sc).T

    Tsc = np.zeros((4, 4))
    Tsc[0:3, 0:3] = R_sc
    Tsc[0:3, 3:] = t_sc
    Tsc[3, 3] = 1

    Tcs = np.linalg.inv(Tsc)

    v_s, w_s = p.getBaseVelocity(robot_id)
    v_s = np.array(v_s)
    w_s = np.array(w_s)
    w_c = R_cs @ w_s
    v_c = R_cs @ v_s

    return {"t_sc": t_sc,
            "q_sc": q_sc,
            "w_c": w_c,
            "v_c": v_c,
            "Tsc": Tsc,
            "Tcs": Tcs}


pose_control = get_pose_control()["lambda"]

for T_st in l_T_st:
    s = get_state()

    q_ts = T_st["q_st"].conj()
    q_tc = q_ts * s["q_sc"]
    t_diff = np.linalg.norm(s["t_sc"] - T_st["t_st"])

    while q_tc.w < 0.99 or t_diff > 0.5 or np.linalg.norm(s["w_c"]) > 0.1 or np.linalg.norm(s["v_c"]) > 0.1:
        s = get_state()
        q_ts = T_st["q_st"].conj()
        q_tc = q_ts * s["q_sc"]
        t_diff = np.linalg.norm(s["t_sc"].squeeze() - T_st["t_st"])

        F_c = pose_control(s["t_sc"], as_float_array(s["q_sc"]),
                           s["w_c"], s["v_c"],
                           T_st["t_st"], as_float_array(T_st["q_st"]), ctrl_gains).squeeze()

        ftz = get_ftz_from_F_c(F_c)
        ftz_max = np.max(np.abs(ftz))
        ftz_lim = 40

        assert(ftz_max < ftz_lim)

        if ftz_max > ftz_lim:
            ftz = ftz * ftz_lim / ftz_max

        for link_idx, ftz_i in enumerate(np.nditer(ftz)):
            # Apply control forces
            p.applyExternalForce(robot_id, link_idx, [0, 0, ftz_i], [0, 0, 0], flags=p.LINK_FRAME)

            # Apply damping from water

            # Visualize
            p_zero_t = np.array([0, 0, 0, 1])
            p_fz_t = np.array([0, 0, ftz_i, 1])
            Tct = brov2["robot"]["joint"][link_idx]["Tct"]
            Tsc = s["Tsc"]
            p_zero_s = Tsc @ Tct @ p_zero_t
            p_fz_s = Tsc @ Tct @ p_fz_t

            db_graph["ftz"][link_idx] = p.addUserDebugLine(p_zero_s.tolist()[0:3], p_fz_s.tolist()[0:3], [0, 1, 0],
                                                           parentObjectUniqueId=robot_id,
                                                           parentLinkIndex=link_idx,
                                                           replaceItemUniqueId=db_graph["ftz"][link_idx])

        db_graph["w_c"] = p.addUserDebugLine([0, 0, 0], list(s["w_c"]), [1, 0, 0],
                                             parentObjectUniqueId=robot_id,
                                             parentLinkIndex=-1,
                                             replaceItemUniqueId=db_graph["w_c"])

        p.stepSimulation()

        time.sleep(1. / 240.)

p.disconnect()

# TODO write video
# import cv2
# import pyautogui
#
# out = cv2.VideoWriter('outpy.avi', cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 10, (640, 480))
#
# for i in range(100):
#
#     image = pyautogui.screenshot()
#     frame = cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)
#
#     out.write(frame)
#     # cv2.imshow('frame',frame)
#
# out.release()
# cv2.destroyAllWindows()
#
# exit(0)
