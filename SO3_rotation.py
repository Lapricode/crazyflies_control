import numpy as np
import tolerances


'''
tau = theta * u (angle-axis representation) is the cartesian space element, where theta \in R is the angle in radians and u \in R^3 unit vector along the axis of rotation
R = I + sin(theta) * u_hat + (1 - cos(theta)) * u_hat^2 is the group element
tau_hat = theta * u_hat is the algebra element, where u_hat = [0, -u3, u2; u3, 0, -u1; -u2, u1, 0]
'''
tol = tolerances.small_case_tol  # tolerance for numerical issues

def group_element(tau):
    theta, u = decompose_cartesian_element(tau)
    u_hat = vec_hat(u)
    R = np.eye(3) + np.sin(theta) * u_hat + (1 - np.cos(theta)) * u_hat @ u_hat
    return R

def group_identity():
    return np.eye(3)

def group_composition(R1, R2):
    return R1 @ R2

def group_inverse(R):
    return R.T

def group_action(R, x):
    return R @ x

def algebra_element(tau):
    theta, u = decompose_cartesian_element(tau)
    tau_hat = theta * vec_hat(u)
    return tau_hat

def compose_cartesian_element(theta, u):
    return theta * u

def decompose_cartesian_element(tau):
    theta = np.linalg.norm(tau)
    if abs(theta) < tol:
        return 0., np.array([0., 0., 1.])
    u = tau / theta
    return theta, u

def hat(tau):
    theta, u = decompose_cartesian_element(tau)
    tau_hat = theta * vec_hat(u)
    return tau_hat

def vee(tau_hat):
    tau = np.block([tau_hat[2, 1], tau_hat[0, 2], tau_hat[1, 0]])
    theta = np.linalg.norm(tau)
    if abs(theta) < tol:
        return np.zeros((3, 1))
    u = tau / theta
    return compose_cartesian_element(theta, u)

def exp(tau_hat):
    tau = np.block([tau_hat[2, 1], tau_hat[0, 2], tau_hat[1, 0]])
    theta = np.linalg.norm(tau)
    if abs(theta) < tol:
        return np.eye(3)
    u = tau / theta
    u_hat = vec_hat(u)
    R = np.eye(3) + np.sin(theta) * u_hat + (1 - np.cos(theta)) * u_hat @ u_hat
    return R

def log(R):
    theta = np.arccos((np.trace(R) - 1.) / 2.)
    if abs(theta) < tol:
        return (R - R.T) / 2.
    tau_hat = theta * (R - R.T) / (2. * np.sin(theta))
    return tau_hat
    # theta = np.arccos(np.clip((np.trace(R) - 1.) / 2., -1.0, 1.0))
    # if np.isclose(theta, 0., rtol = tol, atol = tol):
    #     return (R - R.T) / 2.
    # elif np.isclose(theta, np.pi, rtol = tol, atol = tol):
    #     r00, r01, r02 = R[0, 0], R[0, 1], R[0, 2]
    #     r10, r11, r12 = R[1, 0], R[1, 1], R[1, 2]
    #     r20, r21, r22 = R[2, 0], R[2, 1], R[2, 2]
    #     if not np.isclose(r22, -1., rtol = tol, atol = tol):
    #         multiplier = theta / np.sqrt(2. * (1. + r22))
    #         return multiplier * np.array([r02, r12, 1. + r22])
    #     elif not np.isclose(r11, -1., rtol = tol, atol = tol):
    #         multiplier = theta / np.sqrt(2. * (1. + r11))
    #         return multiplier * np.array([r01, 1. + r11, r21])
    #     elif not np.isclose(r00, -1., rtol = tol, atol = tol):
    #         multiplier = theta / np.sqrt(2. * (1. + r00))
    #         return multiplier * np.array([1. + r00, r10, r20])
    # tau_hat = theta * (R - R.T) / (2. * np.sin(theta))
    # return tau_hat

def Exp(tau):
    theta, u = decompose_cartesian_element(tau)
    u_hat = vec_hat(u)
    R = np.eye(3) + np.sin(theta) * u_hat + (1. - np.cos(theta)) * u_hat @ u_hat
    return R

def Log(R):
    theta = np.arccos((np.trace(R) - 1.) / 2.)
    if abs(theta) < tol:
        return compose_cartesian_element(theta, np.array([0., 0., 1.]))
    tau_hat = theta * (R - R.T) / (2. * np.sin(theta))
    u = np.block([tau_hat[2, 1], tau_hat[0, 2], tau_hat[1, 0]]) / theta
    return compose_cartesian_element(theta, u)
    # theta = np.arccos(np.clip((np.trace(R) - 1.) / 2., -1.0, 1.0))
    # if np.isclose(theta, 0., rtol = tol, atol = tol):
    #     return np.zeros(3,)
    # elif np.isclose(theta, np.pi, rtol = tol, atol = tol):
    #     r00, r01, r02 = R[0, 0], R[0, 1], R[0, 2]
    #     r10, r11, r12 = R[1, 0], R[1, 1], R[1, 2]
    #     r20, r21, r22 = R[2, 0], R[2, 1], R[2, 2]
    #     if not np.isclose(r22, -1., rtol = tol, atol = tol):
    #         multiplier = theta / np.sqrt(2. * (1. + r22))
    #         return multiplier * np.array([r02, r12, 1. + r22])
    #     elif not np.isclose(r11, -1., rtol = tol, atol = tol):
    #         multiplier = theta / np.sqrt(2. * (1. + r11))
    #         return multiplier * np.array([r01, 1. + r11, r21])
    #     elif not np.isclose(r00, -1., rtol = tol, atol = tol):
    #         multiplier = theta / np.sqrt(2. * (1. + r00))
    #         return multiplier * np.array([1. + r00, r10, r20])
    # tau_hat = theta * (R - R.T) / (2. * np.sin(theta))
    # u = np.block([tau_hat[2, 1], tau_hat[0, 2], tau_hat[1, 0]]) / theta
    # return compose_cartesian_element(theta, u)

def plus_right(R, tau):
    return R @ Exp(tau)

def plus_left(R, tau):
    return Exp(tau) @ R

def minus_right(R1, R2):
    return Log(R2.T @ R1)

def minus_left(R1, R2):
    return Log(R1 @ R2.T)

def adjoint(R):
    return R

def jacobian_inverse(R):
    return -R

def jacobian_composition_1(R1, R2):
    return R2.T

def jacobian_composition_2(R1, R2):
    return np.eye(3)

def jacobian_right(tau):
    theta, u = decompose_cartesian_element(tau)
    u_hat = vec_hat(u)
    return np.eye(3) - (1 - np.cos(theta)) / theta * u_hat + (theta - np.sin(theta)) / theta * u_hat @ u_hat if abs(theta) >= tol else np.eye(3)

def jacobian_right_inverse(tau):
    theta, u = decompose_cartesian_element(tau)
    u_hat = vec_hat(u)
    return np.eye(3) + 0.5 * theta * u_hat + (1. - 0.5 * theta * (1. + np.cos(theta)) / np.sin(theta)) * u_hat @ u_hat if abs(theta) >= tol else np.eye(3)

def jacobian_left(tau):
    theta, u = decompose_cartesian_element(tau)
    u_hat = vec_hat(u)
    return np.eye(3) + (1 - np.cos(theta)) / theta * u_hat + (theta - np.sin(theta)) / theta * u_hat @ u_hat if abs(theta) >= tol else np.eye(3)

def jacobian_left_inverse(tau):
    theta, u = decompose_cartesian_element(tau)
    u_hat = vec_hat(u)
    return np.eye(3) - 0.5 * theta * u_hat + (1. - 0.5 * theta * (1. + np.cos(theta)) / np.sin(theta)) * u_hat @ u_hat if abs(theta) >= tol else np.eye(3)

def jacobian_plus_right_1(R, tau):
    theta, u = decompose_cartesian_element(tau)
    u_hat = vec_hat(u)
    R = np.eye(3) + np.sin(theta) * u_hat + (1. - np.cos(theta)) * u_hat @ u_hat
    return R.T

def jacobian_plus_right_2(R, tau):
    return jacobian_right(tau)

def jacobian_minus_right_1(R1, R2):
    return jacobian_right_inverse(Log(R2.T @ R1))

def jacobian_minus_right_2(R1, R2):
    return -jacobian_left_inverse(Log(R2.T @ R1))

def jacobian_rotation_action_1(R, x):
    return -R @ vec_hat(x)

def jacobian_rotation_action_2(R, x):
    return R

def qR_quat(R):
    w = np.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
    x = np.sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])) / 2.0
    y = np.sqrt(max(0.0, 1.0 - R[0, 0] + R[1, 1] - R[2, 2])) / 2.0
    z = np.sqrt(max(0.0, 1.0 - R[0, 0] - R[1, 1] + R[2, 2])) / 2.0
    x = np.copysign(x, R[2, 1] - R[1, 2])
    y = np.copysign(y, R[0, 2] - R[2, 0])
    z = np.copysign(z, R[1, 0] - R[0, 1])
    qR = np.array([w, x, y, z])
    return qR

def vec_hat(v):
    return np.block([[0., -v[2], v[1]], [v[2], 0., -v[0]], [-v[1], v[0], 0.]])
