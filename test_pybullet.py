import pybullet as p
import pybullet_data
import time
from enum import Enum

import numpy as np
from quaternion import from_euler_angles
from quaternion import as_rotation_matrix

import quaternion as nq

from pyxacro import get_robot
from motion_model import get_get_ftz_from_F_c


class ControlType(Enum):
    TORQUE = 1
    FORCE = 2


get_ftz_from_F_c = get_get_ftz_from_F_c()["lambda"]

physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0, 0, 0)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0, 0, 1]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])

urdf_file_path = "../bluerov_ffg/urdf/brov2.urdf"

brov2 = get_robot(urdf_file_path)

boxId = p.loadURDF(urdf_file_path, cubeStartPos, cubeStartOrientation, flags=p.URDF_USE_INERTIA_FROM_FILE)

k = np.array([
    brov2["robot"]["link"][0]["inertial"]["inertia"]["@ixx"],
    brov2["robot"]["link"][0]["inertial"]["inertia"]["@iyy"],
    brov2["robot"]["link"][0]["inertial"]["inertia"]["@izz"]
], dtype=np.float64)

h = k

h = h * 200.0
k = k * 1000.0

l_q_st = [
    from_euler_angles(np.pi / 4, np.pi / 3, 0),
    from_euler_angles(-np.pi / 4, np.pi / 3, 0),
    from_euler_angles(-np.pi / 4, -np.pi / 3, 0),
    from_euler_angles(np.pi / 4, -np.pi / 3, 0),
    ]


p.addUserDebugLine([0, 0, 0], [1, 0, 0], [1, 0, 0], parentObjectUniqueId=boxId, parentLinkIndex=-1)
p.addUserDebugLine([0, 0, 0], [0, 1, 0], [0, 1, 0], parentObjectUniqueId=boxId, parentLinkIndex=-1)
p.addUserDebugLine([0, 0, 0], [0, 0, 1], [0, 0, 1], parentObjectUniqueId=boxId, parentLinkIndex=-1)

# s - origin
# t - target
# c - center of gravity


def get_state():
    t_sc, q_sc = p.getBasePositionAndOrientation(boxId)
    t_sc = np.array(t_sc)
    q_sc = nq.quaternion(q_sc[3], q_sc[0], q_sc[1], q_sc[2])
    R_bs = as_rotation_matrix(q_sc).T
    v_s, w_s = p.getBaseVelocity(boxId)
    v_s = np.array(v_s)
    w_s = np.array(w_s)
    w_c = R_bs @ w_s
    v_c = R_bs @ v_s
    return q_sc, w_c


ctrl = ControlType.FORCE
# ctrl = ControlType.TORQUE

for q_st in l_q_st:
    q_sc, w_c = get_state()

    q_ts = q_st.conj()
    q_tc = q_ts * q_sc

    while q_tc.w < 0.999 or np.linalg.norm(w_c) > 0.1:

        q_sc, w_c = get_state()
        q_tc = q_ts * q_sc

        M_c = -h * w_c - k * q_tc.w * q_tc.vec

        if ctrl == ControlType.TORQUE:
            M_c[0] = np.sign(M_c[0]) * np.minimum(np.abs(M_c[0]), 3)
            M_c[1] = np.sign(M_c[1]) * np.minimum(np.abs(M_c[1]), 3)
            M_c[2] = np.sign(M_c[2]) * np.minimum(np.abs(M_c[2]), 3)
            p.applyExternalTorque(boxId, -1, M_c, flags=p.WORLD_FRAME)  # TODO bug in pybullet. WORLD_FRAME and link frame are inverted https://github.com/bulletphysics/bullet3/issues/1949
        elif ctrl == ControlType.FORCE:
            ftz = get_ftz_from_F_c(np.concatenate((M_c, np.zeros((3,)))))
            for link_idx, ftz_i in enumerate(ftz):
                p.applyExternalForce(boxId, link_idx, [0, 0, ftz_i], [0, 0, 0], flags=p.LINK_FRAME)
        else:
            pass

        p.stepSimulation()

        time.sleep(1./240.)

p.disconnect()


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