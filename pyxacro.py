from pprint import pprint
import xmltodict as xmltd
import numpy as np
from quaternion import from_euler_angles
from quaternion import as_rotation_matrix


def get_robot(urdf_file_path):
    with open(urdf_file_path, 'r') as f:
        rob = xmltd.parse(f.read())

        base_link = rob["robot"]["link"][0]["inertial"]
        base_link["mass"]["@value"] = np.float64(base_link["mass"]["@value"])

    for key in base_link["inertia"].keys():
        base_link["inertia"][key] = np.float64(base_link["inertia"][key])
        base_link["origin"]["@xyz"] = np.float64(base_link["origin"]["@xyz"].split(" "))
        base_link["origin"]["@rpy"] = np.float64(base_link["origin"]["@rpy"].split(" "))

        q_bc = from_euler_angles(base_link["origin"]["@rpy"][2],
                                 base_link["origin"]["@rpy"][1],
                                 base_link["origin"]["@rpy"][0])

        r_bc = as_rotation_matrix(q_bc.normalized())

        pc = base_link["origin"]["@xyz"].reshape((3, 1))

        # pc[2] += 0.03  # Uncomment if ideal alignment with COG needed.

        base_link["origin"]["Tbc"] = np.concatenate((np.concatenate((r_bc, pc), axis=1),
                                                     np.array([0, 0, 0, 1], ndmin=2)), axis=0)

        base_link["origin"]["Tcb"] = np.linalg.inv(base_link["origin"]["Tbc"])

        for joint in rob["robot"]["joint"]:
            joint["origin"]["@xyz"] = np.float64(joint["origin"]["@xyz"].split(" "))
            joint["origin"]["@rpy"] = np.float64(joint["origin"]["@rpy"].split(" "))

            q_bt = from_euler_angles(joint["origin"]["@rpy"][2],
                                  joint["origin"]["@rpy"][1],
                                  joint["origin"]["@rpy"][0])

            r_bt = as_rotation_matrix(q_bt.normalized())

            pt = joint["origin"]["@xyz"].reshape((3, 1))
            # b - body
            # t - thruster
            # c - center of gravity
            joint["Tbt"] = np.concatenate((np.concatenate((r_bt, pt), axis=1),
                                           np.array([0, 0, 0, 1], ndmin=2)), axis=0)

            joint["Tct"] = base_link["origin"]["Tcb"] @ joint["Tbt"]

        return rob


if __name__ == '__main__':
    np.set_printoptions(precision=3)
    th = get_robot("../bluerov_ffg/urdf/brov2.urdf")
    pprint(th)
