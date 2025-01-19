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
    
    return adjusted_com

def calculate_com(multiplier):
    with open("com.csv", "r") as f:
        data = [line.split(",") for line in f]

    i = 1
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
        ans = adjust(mass, com, com_offset, quat, np_motors)
        print(ans[0])
        print(ans[1])
        print(ans[2])
        print()
        i += 1

def main():
    multiplier = 1.0
    calculate_com(multiplier)

if __name__ == "__main__":
    main()






