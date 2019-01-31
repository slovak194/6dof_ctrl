from pprint import pprint
import xml.etree.ElementTree
import numpy as np
from quaternion import from_euler_angles
from quaternion import as_rotation_matrix


def parse_xacro_expr(expr: str, prop_dict: dict):
    els = expr.split(" ")
    nums = []
    for elem in els:
        elem = elem.replace("${", "")
        elem = elem.replace("}", "")
        for key, value in prop_dict.items():
            elem = elem.replace(key, value)
        if type(elem) is float:
            nums.append(np.float64(elem))
        elif type(elem) is str:
            nums.append(np.float64(eval(elem)))
        else:
            assert False

    return np.array(nums)


def get_thrusters_poses(xacro_file_path):
    e = xml.etree.ElementTree.parse(xacro_file_path).getroot()
    props = {}

    for el in e.findall("{http://www.ros.org/wiki/xacro}property"):
        if type(el.get("value")) is not str:
            print(el.get("value"))
        props[el.get("name")] = el.get("value")

    thrusters = {}

    for el in e.findall("{http://www.ros.org/wiki/xacro}thruster_link"):
        thrusters[el.get("name")] = {}
        xyz = parse_xacro_expr(el.get("xyz"), props)
        rpy = parse_xacro_expr(el.get("rpy"), props)

        r = as_rotation_matrix(from_euler_angles(rpy[0], rpy[1], rpy[2]))
        p = xyz.reshape((3, 1))
        # b - body
        # t - thruster
        thrusters[el.get("name")]["Tbt"] = np.concatenate((np.concatenate((r, p), axis=1),
                                                            np.array([0, 0, 0, 1], ndmin=2)), axis=0)

    return thrusters


if __name__ == '__main__':
    np.set_printoptions(precision=3)
    th = get_thrusters_poses("../bluerov_ffg/urdf/brov2.xacro")
    pprint(th)
