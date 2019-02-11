import pybullet as p
import pybullet_data
import time

import numpy as np
from quaternion import from_euler_angles
from quaternion import as_rotation_matrix

import quaternion as nq

from pyxacro import get_robot

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0, 0, 0)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0, 0, 1]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])

urdf_file_path = "../bluerov_ffg/urdf/brov2.urdf"


brov2 = get_robot(urdf_file_path)


boxId = p.loadURDF(urdf_file_path, cubeStartPos, cubeStartOrientation, flags=p.URDF_USE_INERTIA_FROM_FILE)


h = np.array([
    brov2["robot"]["link"][0]["inertial"]["inertia"]["@ixx"],
    brov2["robot"]["link"][0]["inertial"]["inertia"]["@iyy"],
    brov2["robot"]["link"][0]["inertial"]["inertia"]["@izz"]
], dtype=np.float64)

k = h

q_target = from_euler_angles(np.pi/6, 0, 0)

for i in range(10000):

    t, q_self = p.getBasePositionAndOrientation(boxId)
    t = np.array(t)
    q_self = nq.quaternion(*q_self)
    v, w = p.getBaseVelocity(boxId)
    v = np.array(v)
    w = np.array(w)

    q_diff = q_target.conj() * q_self

    M = -h*w - k*q_diff.w*q_diff.vec
    #
    # k_i = I_i
    # h_i = I_i

    # p.applyExternalForce(boxId, 4, [0, 0, 40], [0, 0, 0], flags=p.LINK_FRAME)
    # p.applyExternalForce(boxId, 5, [0, 0, 40], [0, 0, 0], flags=p.LINK_FRAME)
    p.applyExternalTorque(boxId, 4, M, flags=p.LINK_FRAME)

    p.stepSimulation()

    # time.sleep(1./240.)

cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos, cubeOrn)
p.disconnect()
