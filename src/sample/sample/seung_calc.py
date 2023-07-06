import numpy as np


def Q2M(o4x1):
    q0, q1, q2, q3 = (o4x1[0], o4x1[1], o4x1[2], o4x1[3])

    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1

    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]])

    return rot_matrix


def Transform(rotation_matrix, point_x, point_y, point_z, tx, ty, tz):
    result = np.zeros((1, 3))
    point = np.zeros([[point_x], [point_y], [point_z]])
    tr = np.array([[tx], [ty], [tz]])

    result = rotation_matrix @ point + tr
    return result


def C2W(orientation_4x1, points_Nx3x1, pos_1x3):
    rot3x3 = Q2M(orientation_4x1)
    result = np.matmul(rot3x3, points_Nx3x1) + pos_1x3
    return result


def refine_resol(x, y, pixel_x, pixel_y):
    want_pixel_index = (pixel_x) * (y - 1) + x
    return want_pixel_index
