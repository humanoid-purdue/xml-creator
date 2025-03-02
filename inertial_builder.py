import pandas
import numpy as np
import xml.etree.ElementTree as ET

M_MASS = 0.53
TAB = "  "
def inertial_dict(pd, approximate_moi = False, weight_factor = 1.0):
    def row_in_dict(row):
        root_pos = np.array([row["root_pos_x"],
                             row["root_pos_y"],
                             row["root_pos_z"]])
        material_mass = row["material_mass"]
        com_raw = np.array([row["com_pos_x"],
                            row["com_pos_y"],
                            row["com_pos_z"]])
        num_motors = round(row["num_motors"])
        com = com_raw * material_mass * weight_factor
        for c in range(num_motors):
            m_n = "motor{}_pos".format(c + 1)
            motor_pos = np.array([row["{}_x".format(m_n)],
                                  row["{}_y".format(m_n)],
                                  row["{}_z".format(m_n)]])
            com = com + motor_pos * M_MASS
        mass = material_mass * weight_factor + num_motors * M_MASS
        com = com / mass
        moi = np.zeros([3, 3])
        sphere = 0.4 * mass * (np.linalg.norm(com) * 0.5) ** 2
        moi[0, 0] = sphere
        moi[1, 1] = sphere
        moi[2, 2] = sphere
        if not approximate_moi:
            for i in range(3):
                for k in range(3):
                    moi[i, k] = row["moi{}{}".format(i, k)]
        return root_pos, {"mass": mass, "com": com, "moi": moi, "torque": row["joint_torque"]}

    def mirror_part(in_dict):
        in_dict["com"] = in_dict["com"] * np.array([1, -1, 1])
        in_dict["moi"] = in_dict["moi"] * np.array([[1, -1, 1],
                                                    [-1, 1, -1],
                                                    [1, -1, 1]])
        return in_dict
    inertial_dict = {"pelvis": row_in_dict(pd.loc[0])}
    for c in range(6):
        name = pd.loc[c + 1]["part_name"]
        root_pos, in_dict = row_in_dict(pd.loc[c+1])
        inertial_dict[name] = (root_pos, in_dict)
        r_name = "r" + name[1:]
        inertial_dict[r_name] = (root_pos * np.array([1, -1, 1]),
                                 mirror_part(in_dict))
    return inertial_dict


def pd2mjxml(file_name, write_name, robot_name):
    def replace_tree_inertia(tree, in_dict):
        tree.set("pos", "{} {} {}".format(in_dict[0][0],
                                     in_dict[0][1],
                                     in_dict[0][2]))
        tree[0].set("pos", "{} {} {}".format(in_dict[1]["com"][0],
                                           in_dict[1]["com"][1],
                                           in_dict[1]["com"][2]))
        tree[0].set("mass", "{}".format(in_dict[1]["mass"]))
        tree[0].set("fullinertia", "{} {} {} {} {} {}".format(
            in_dict[1]["moi"][0, 0], in_dict[1]["moi"][1, 1], in_dict[1]["moi"][2, 2],
            in_dict[1]["moi"][0, 1], in_dict[1]["moi"][0, 2], in_dict[1]["moi"][1, 2]
        ))
    pd = pandas.read_excel(file_name)
    ind = inertial_dict(pd, approximate_moi = True)

    tree = ET.parse('mujoco_template.xml')
    tree.getroot().set("model", robot_name)
    pelvis = tree.getroot()[6][1]
    l_elements = {}
    l_elements["l_hip_pitch"] = pelvis[3]
    l_elements["l_hip_roll"] = l_elements["l_hip_pitch"][3]
    l_elements["l_hip_yaw"] = l_elements["l_hip_roll"][3]
    l_elements["l_knee"] = l_elements["l_hip_yaw"][3]
    l_elements["l_foot_pitch"] = l_elements["l_knee"][3]
    l_elements["l_foot_roll"] = l_elements["l_foot_pitch"][3]

    r_elements = {}
    r_elements["r_hip_pitch"] = pelvis[4]
    r_elements["r_hip_roll"] = r_elements["r_hip_pitch"][3]
    r_elements["r_hip_yaw"] = r_elements["r_hip_roll"][3]
    r_elements["r_knee"] = r_elements["r_hip_yaw"][3]
    r_elements["r_foot_pitch"] = r_elements["r_knee"][3]
    r_elements["r_foot_roll"] = r_elements["r_foot_pitch"][3]

    replace_tree_inertia(pelvis, ind["pelvis"])
    n_list = ["_hip_pitch", "_hip_roll", "_hip_yaw", "_knee", "_foot_pitch", "_foot_roll"]
    for c in range(6):
        replace_tree_inertia(l_elements["l" + n_list[c]], ind["l" + n_list[c]])
        replace_tree_inertia(r_elements["r" + n_list[c]], ind["r" + n_list[c]])
        tree.getroot()[7][c].set("forcerange", "{} {}".format(
            ind["l" + n_list[c]][1]["torque"] * -0.5,
            ind["l" + n_list[c]][1]["torque"] * 0.5
        ))
        tree.getroot()[7][c + 6].set("forcerange", "{} {}".format(
            ind["r" + n_list[c]][1]["torque"] * -0.5,
            ind["r" + n_list[c]][1]["torque"] * 0.5
        ))
        tree.getroot()[7][c + 12].set("forcerange", "{} {}".format(
            ind["l" + n_list[c]][1]["torque"] * -0.5,
            ind["l" + n_list[c]][1]["torque"] * 0.5
        ))
        tree.getroot()[7][c + 18].set("forcerange", "{} {}".format(
            ind["r" + n_list[c]][1]["torque"] * -0.5,
            ind["r" + n_list[c]][1]["torque"] * 0.5
        ))
    tree.write(write_name, encoding = "utf-8", xml_declaration = True)


if __name__ == "__main__":
    xlsx_name = "nemo3_inertial.xlsx"
    robot_name = "nemo3"
    pd2mjxml(xlsx_name, "{}.xml".format(robot_name), robot_name)

