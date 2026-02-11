from scipy.spatial.transform import Rotation as R
import numpy as np

def calculate_angle_between_euler_angles(first_euler_angle : list, second_euler_angle : list):
    first = R.from_euler('xyz', first_euler_angle, degrees=True)
    second = R.from_euler('xyz', second_euler_angle, degrees=True)

    relative = first.inv() * second
    angle = np.linalg.norm(relative.as_rotvec())

    return np