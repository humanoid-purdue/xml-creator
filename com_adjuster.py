import numpy as np
from scipy.spatial.transform import Rotation as R

def adjust(mass, com, com_offset, quat, motors):
    translated_com = com - com_offset

    r  = R.from_quat(quat)
    rotated_com = np.matmul(r.as_matrix(), translated_com)

    adjusted_com = rotated_com * mass
    total_mass = mass
    for motor in motors:
        total_mass += 0.53
        adjusted_com += motor * 0.53
    
    adjusted_com /= total_mass
    
    return adjusted_com

mass = 1.165732
com = np.array([0.0762, 0.021336, 0])
com_offset = np.array([0.0762, -0.01905, 0])

quat = np.array([0.5, 0.5, 0.5, 0.5])
motors = np.array([[0, -0.05715, 0], [0, 0.05715, 0]])

print(adjust(mass, com, com_offset, quat, motors))





