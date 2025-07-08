import numpy as np
from scipy.spatial.transform import Rotation


def get_robot_pose_matrix(x, y, z, rx, ry, rz):
    R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T

def transform_to_base(camera_coords, gripper2cam):
    from DSR_ROBOT2 import get_current_posx
    coord = np.append(np.array(camera_coords), 1)  # Homogeneous coordinate
    base2gripper = get_robot_pose_matrix(*get_current_posx()[0])
    base2cam = base2gripper @ gripper2cam
    td_coord = np.dot(base2cam, coord)
    
    return td_coord[:3]

