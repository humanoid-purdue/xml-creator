import numpy as np
from scipy.spatial.transform import Rotation as R

def adjust(mass, com, com_offset, quat, extra_masses):
    translated_com = com - com_offset

    r  = R.from_quat(quat)
    rotated_com = np.matmul(r.as_matrix(), translated_com)

    adjusted_com = rotated_com * mass
    total_mass = mass
    for extra_mass in extra_masses:
        total_mass += extra_mass[0]
        adjusted_com += extra_mass[1:] * extra_mass[0]
    
    adjusted_com /= total_mass
    
    return adjusted_com, total_mass

def dict2inertial(info_dict):
    #Example: <inertial pos="0 0 -0.07605" quat="1 0 -0.000405289 0" mass="2.86" diaginertia="0.0079143 0.0069837 0.0059404"/>
    pos_str = "<inertial pos =\"{} {} {}\" ".format(info_dict["com"][0], info_dict["com"][1], info_dict["com"][2])
    quat = "quat=\"1 0 0 0\" "
    mass = "mass=\"{}\" ".format(info_dict["mass"])
    inert = info_dict["inertia"]
    diaginertia = "diaginertia=\"{} {} {}\"/>".format(inert[0, 0], inert[1, 1], inert[2, 2])
    return pos_str + quat + mass + diaginertia


def calculate_com(multiplier):
    with open("com.csv", "r") as f:
        data = [line.split(",") for line in f]

    i = 1
    info_dict = {}
    while i < len(data):
        line = data[i]
        name = line[0]
        mass = float(line[1]) * multiplier
        com = np.array([float(x) for x in line[2:5]])
        com_offset = np.array([float(x) for x in line[5:8]])
        quat = np.array([float(x) for x in line[8:12]])
        num_extra_masses = int(line[12])
        extra_masses = []
        for _ in range(num_extra_masses):
            i += 1
            extra_masses.append([float(x) for x in data[i][1:5]])
        np_motors = np.array(extra_masses)

        print(f"{name}:")
        com_new, mass2 = adjust(mass, com, com_offset, quat, np_motors)
        com_new = np.array(com_new)
        if name == "pelvis":
            pm2 = np.array([0, 0, 0.3])
            com_new = (com_new * mass2 + pm2 * 10) / (mass2 + 10)
            mass2 += 5

        radius = np.linalg.norm(com_new) * ( 0.4 )
        inert_sphere = np.eye(3) * (0.4 * mass2 * radius**2)
        #inert_pat = np.linalg.norm(com_new)**2 * np.eye(3) - np.matmul(np.transpose(com_new), com_new)
        inert = inert_sphere # + inert_pat * mass2
        info_dict[name] = {"com": com_new, "mass": mass2, "inertia": inert}
        print(dict2inertial(info_dict[name]))
        i += 1

def main():
    multiplier = 0.2
    calculate_com(multiplier)

if __name__ == "__main__":
    main()






