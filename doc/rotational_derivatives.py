import numpy as np
import modern_robotics as mr
import pyquaternion

from acrolib.geometry import rot_x, rot_y, rot_z, rotation_matrix_to_rpy, rpy_to_rot_mat

def rpyToOmega(rpy):
    cosx = np.cos(rpy[0])
    sinx = np.sin(rpy[0])
    cosy = np.cos(rpy[1])
    siny = np.sin(rpy[1])

    return np.array([[1, 0, siny], [0, cosx, -cosy * sinx], [0, sinx, cosx * siny]])

def applyRotation(angle, axis):
    assert(mr.NearZero(np.linalg.norm(axis) - 1))
    return mr.MatrixExp3(mr.VecToso3(axis * angle))


def numericalDerivativeRPY(rotation, omega):
    h = 1e-6

    R = rotation
    R_plus_h = applyRotation(h, w) @ rotation

    rpy = np.array(rotation_matrix_to_rpy(R))
    rpy_plus_h = np.array(rotation_matrix_to_rpy(R_plus_h))

    return (rpy_plus_h - rpy) / h

def analyticalDerivativeRPY(rotation, omega):
    rpy = rotation_matrix_to_rpy(rotation)
    return np.linalg.inv(rpyToOmega(rpy)) @ omega

R1 = pyquaternion.Quaternion.random().rotation_matrix
# R1 = np.eye(3)
print(rotation_matrix_to_rpy(R1))
# w = np.array([1.0, 0, 0])

w = np.random.uniform(size=(3,))
w = w / np.linalg.norm(w)

print(numericalDerivativeRPY(R1, w))

print(analyticalDerivativeRPY(R1, w))
