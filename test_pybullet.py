import pybullet as p
import pybullet_data
import time

import numpy as np
from quaternion import from_euler_angles
from quaternion import as_rotation_matrix

import quaternion as nq

from pyxacro import get_robot

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

h = k * 10

q_st = from_euler_angles(np.pi / 1.1,
                         np.pi / 3,
                         0)

q_ts = q_st.conj()

p.addUserDebugLine([0, 0, 0], [1, 0, 0], [1, 0, 0], parentObjectUniqueId=boxId, parentLinkIndex=-1)
p.addUserDebugLine([0, 0, 0], [0, 1, 0], [0, 1, 0], parentObjectUniqueId=boxId, parentLinkIndex=-1)
p.addUserDebugLine([0, 0, 0], [0, 0, 1], [0, 0, 1], parentObjectUniqueId=boxId, parentLinkIndex=-1)

for i in range(1000000):

    t_sb, q_sb = p.getBasePositionAndOrientation(boxId)
    t_sb = np.array(t_sb)
    q_sb = nq.quaternion(q_sb[3], q_sb[0], q_sb[1], q_sb[2])
    R_bs = as_rotation_matrix(q_sb).T
    v_s, w_s = p.getBaseVelocity(boxId)
    v_s = np.array(v_s)
    w_s = np.array(w_s)

    w_b = R_bs @ w_s
    v_b = R_bs @ v_s

    q_tb = q_ts * q_sb

    M_b = -h * w_b - k * q_tb.w * q_tb.vec

    p.applyExternalTorque(boxId, -1, M_b, flags=p.WORLD_FRAME)  # TODO bug in pybullet. WORLD_FRAME and link frame are inverted https://github.com/bulletphysics/bullet3/issues/1949

    p.stepSimulation()

    # time.sleep(1./240.)
    if i % 10 == 0:
        time.sleep(1. / 1000000.0)


p.disconnect()
