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

# p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "brov2.mp4")

planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0, 0, 1]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])

urdf_file_path = "../bluerov_ffg/urdf/brov2.urdf"

brov2 = get_robot(urdf_file_path)

robot_id = p.loadURDF(urdf_file_path, cubeStartPos, cubeStartOrientation, flags=p.URDF_USE_INERTIA_FROM_FILE)

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

db_graph = {}
# dbgraph["x"] = p.addUserDebugLine([0, 0, 0], [1, 0, 0], [1, 0, 0], parentObjectUniqueId=robot_id, parentLinkIndex=-1)
# dbgraph["y"] = p.addUserDebugLine([0, 0, 0], [0, 1, 0], [0, 1, 0], parentObjectUniqueId=robot_id, parentLinkIndex=-1)
# dbgraph["z"] = p.addUserDebugLine([0, 0, 0], [0, 0, 1], [0, 0, 1], parentObjectUniqueId=robot_id, parentLinkIndex=-1)

db_graph["w_c"] = w_c = p.addUserDebugLine([0, 0, 0], [0, 0, 1], [1, 0, 0], parentObjectUniqueId=robot_id,
                                           parentLinkIndex=-1)
db_graph["ftz"] = []

for link_idx in range(0, 6):
    db_graph["ftz"].append(p.addUserDebugLine([0, 0, 0], [0, 0, 1], [0, 1, 0],
                                              parentObjectUniqueId=robot_id,
                                              parentLinkIndex=link_idx))

# s - origin
# t - target
# c - center of gravity


def capture_frame():
    camTargetPos = [0.,0.,0.]
    cameraUp = [0,0,1]
    cameraPos = [1,1,1]
    yaw = 40
    pitch = 10.0

    roll=0
    upAxisIndex = 2
    camDistance = 4
    pixelWidth = 320
    pixelHeight = 240
    nearPlane = 0.01
    farPlane = 1000
    lightDirection = [0,1,0]
    lightColor = [1,1,1]#optional argument
    fov = 60

    viewMatrix = p.computeViewMatrixFromYawPitchRoll(camTargetPos, camDistance, yaw, pitch, roll, upAxisIndex)
    aspect = pixelWidth / pixelHeight
    projectionMatrix = p.computeProjectionMatrixFOV(fov, aspect, nearPlane, farPlane)
    img_arr = p.getCameraImage(pixelWidth, pixelHeight, viewMatrix,projectionMatrix, lightDirection,lightColor)
    w=img_arr[0]
    h=img_arr[1]
    rgb=img_arr[2]
    dep=img_arr[3]
    #print 'width = %d height = %d' % (w,h)
    # reshape creates np array
    np_img_arr = np.reshape(rgb, (h, w, 4))
    np_img_arr = np_img_arr*(1./255.)



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

    return {"q_sc": q_sc, "w_c": w_c, "Tsc": Tsc, "Tcs": Tcs}


ctrl = ControlType.FORCE
# ctrl = ControlType.TORQUE

for q_st in l_q_st:
    s = get_state()

    q_ts = q_st.conj()
    q_tc = q_ts * s["q_sc"]

    prev_q_tc_w = q_tc.w

    while q_tc.w < 0.999 or np.linalg.norm(s["w_c"]) > 0.1:

        s = get_state()
        q_tc = q_ts * s["q_sc"]

        M_c = -h * s["w_c"] - k * q_tc.w * q_tc.vec

        if ctrl == ControlType.TORQUE:
            M_c[0] = np.sign(M_c[0]) * np.minimum(np.abs(M_c[0]), 3)
            M_c[1] = np.sign(M_c[1]) * np.minimum(np.abs(M_c[1]), 3)
            M_c[2] = np.sign(M_c[2]) * np.minimum(np.abs(M_c[2]), 3)
            p.applyExternalTorque(robot_id, -1, M_c, flags=p.WORLD_FRAME)
            # TODO bug in pybullet.
            # TODO WORLD_FRAME and link frame are inverted https://github.com/bulletphysics/bullet3/issues/1949
        elif ctrl == ControlType.FORCE:
            ftz = get_ftz_from_F_c(np.concatenate((M_c, np.zeros((3,)))))
            ftz_max = np.max(np.abs(ftz))
            ftz_lim = 1

            if ftz_max > ftz_lim:
                ftz = ftz*ftz_lim/ftz_max

            for link_idx, ftz_i in enumerate(np.nditer(ftz)):

                p.applyExternalForce(robot_id, link_idx, [0, 0, ftz_i], [0, 0, 0], flags=p.LINK_FRAME)

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
        else:
            pass

        db_graph["w_c"] = p.addUserDebugLine([0, 0, 0], list(s["w_c"]), [1, 0, 0],
                                             parentObjectUniqueId=robot_id,
                                             parentLinkIndex=-1,
                                             replaceItemUniqueId=db_graph["w_c"])

        # if np.abs(prev_q_tc_w - q_tc.w) > 0.01:
        #     prev_q_tc_w = q_tc.w

        p.stepSimulation()
        capture_frame()

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
