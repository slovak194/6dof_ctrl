from pprint import pprint
import xmltodict as xmltd
import numpy as np
from quaternion import from_euler_angles
from quaternion import as_rotation_matrix


def get_robot(urdf_file_path):
    with open(urdf_file_path, 'r') as f:
        rob = xmltd.parse(f.read())

        for joint in rob["robot"]["joint"]:
            joint["origin"]["@xyz"] = np.float64(joint["origin"]["@xyz"].split(" "))
            joint["origin"]["@rpy"] = np.float64(joint["origin"]["@rpy"].split(" "))

            r = as_rotation_matrix(from_euler_angles(joint["origin"]["@rpy"][2],
                                                     joint["origin"]["@rpy"][1],
                                                     joint["origin"]["@rpy"][0]))

            p = joint["origin"]["@xyz"].reshape((3, 1))
            # b - body
            # t - thruster
            joint["Tbt"] = np.concatenate((np.concatenate((r, p), axis=1),
                                           np.array([0, 0, 0, 1], ndmin=2)), axis=0)

        rob["robot"]["link"][0]["inertial"]["origin"]["@xyz"] = \
            np.float64(rob["robot"]["link"][0]["inertial"]["origin"]["@xyz"].split(" "))

        rob["robot"]["link"][0]["inertial"]["origin"]["@rpy"] = \
            np.float64(rob["robot"]["link"][0]["inertial"]["origin"]["@rpy"].split(" "))

        rob["robot"]["link"][0]["inertial"]["mass"]["@value"] = \
            np.float64(rob["robot"]["link"][0]["inertial"]["mass"]["@value"])

        for key in rob["robot"]["link"][0]["inertial"]["inertia"].keys():
            rob["robot"]["link"][0]["inertial"]["inertia"][key] = \
                np.float64(rob["robot"]["link"][0]["inertial"]["inertia"][key])

        return rob


if __name__ == '__main__':
    np.set_printoptions(precision=3)
    th = get_robot("../bluerov_ffg/urdf/brov2.urdf")
    pprint(th)
