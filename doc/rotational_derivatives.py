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

def matrixToAA(rotation):
    return mr.so3ToVec(mr.MatrixLog3(rotation))

def numericalDerivativeRPY(rotation, omega):
    h = 1e-6

    R = rotation
    R_plus_h = applyRotation(h, w) @ rotation

    rpy = np.array(rotation_matrix_to_rpy(R))
    rpy_plus_h = np.array(rotation_matrix_to_rpy(R_plus_h))

    return (rpy_plus_h - rpy) / h

def numericalDerivativeAA(rotation, omega):
    h = 1e-6

    R = rotation
    R_plus_h = applyRotation(h, w) @ rotation

    aa = matrixToAA(R)
    aa_plus_h = matrixToAA(R_plus_h)

    return (aa_plus_h - aa) / h

def analyticalDerivativeRPY(rotation, omega):
    rpy = rotation_matrix_to_rpy(rotation)
    return np.linalg.inv(rpyToOmega(rpy)) @ omega


# def analyticalDerivativeAA(rotation, omega):
#     phi = mr.MatrixLog3(rotation) # skew symmetric matrix
#     t = np.linalg.norm(mr.so3ToVec(phi))

#     E = np.eye(3)

#     A = 1 - 0.5 * t * np.sin(t) / (1 - np.cos(t))
#     E += -0.5 * phi
#     E += phi * phi / t**2 * A

#     return E @ omega

def analyticalDerivativeAA(rotation, omega):
    # phi = mr.MatrixLog3(rotation) # skew symmetric matrix
    # t = np.linalg.norm(mr.so3ToVec(phi))

    aa = matrixToAA(rotation)
    t = np.linalg.norm(aa)
    n = aa / t
    n_skew = mr.VecToso3(n)

    E = np.zeros((4, 3))

    A = -0.5 * np.sin(t) / (1-np.cos(t))

    E[0, :] = n
    E[1:, :] = A * n_skew * n_skew - 0.5 * n_skew

    print(E)

    # print(E[1:, :])


    return E @ omega


R1 = pyquaternion.Quaternion.random().rotation_matrix
# R1 = rot_x(0.5)
# # R1 = np.eye(3)
# print(rotation_matrix_to_rpy(R1))
w = np.array([0.0, 1.0, 0.0])

print(matrixToAA(R1))
print()

print(numericalDerivativeAA(R1, w))
print(analyticalDerivativeAA(R1, w))

# w = np.random.uniform(size=(3,))
# w = w / np.linalg.norm(w)

# print(numericalDerivativeRPY(R1, w))

# print(analyticalDerivativeRPY(R1, w))
