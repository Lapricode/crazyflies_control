import numpy as np
import tolerances


'''
tau = theta * u (angle-axis representation) is the cartesian space element, where theta \in R is the angle in radians and u \in R^3 is the unit vector along the axis of rotation (pure qaternion)
q = [cos(theta/2); sin(theta/2) * u] is the group element, where u \in R^3 is the unit vector along the axis of rotation (pure quaternion)
tau_hat = theta * u_hat is the algebra element, where u_hat = [0, -u3, u2; u3, 0, -u1; -u2, u1, 0]
'''
tol = tolerances.small_case_tol  # tolerance for numerical issues

def group_element(tau):
    theta, u = decompose_cartesian_element(tau)
    q = np.array([np.cos(theta / 2), np.sin(theta / 2) * u[0], np.sin(theta / 2) * u[1], np.sin(theta / 2) * u[2]])
    return q

def group_identity():
    return np.array([1., 0., 0., 0.])

def group_composition(q1, q2):
    w1, x1, y1, z1 = q1[0], q1[1], q1[2], q1[3]
    w2, x2, y2, z2 = q2[0], q2[1], q2[2], q2[3]
    q = np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                    w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                    w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                    w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])
    return q

def group_inverse(q):
    w, x, y, z = q[0], q[1], q[2], q[3]
    return np.array([w, -x, -y, -z])

def group_action(q, x):
    return group_composition(group_composition(q, np.block([0., x])), group_inverse(q))[1:]

def algebra_element(tau):
    tau_hat = tau / 2.
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
    tau_hat = tau / 2.
    return tau_hat

def vee(tau_hat):
    tau = 2. * tau_hat
    return tau

def exp(tau_hat):
    tau = 2. * tau_hat
    theta = np.linalg.norm(tau)
    if abs(theta) < tol:
        return np.array([1., 0., 0., 0.])
    u = tau / theta
    q = np.array([np.cos(theta / 2), np.sin(theta / 2) * u[0], np.sin(theta / 2) * u[1], np.sin(theta / 2) * u[2]])
    return q

def log(q):
    w, x, y, z = q[0], q[1], q[2], q[3]
    v = np.array([x, y, z])
    v_norm = np.linalg.norm(v)
    if abs(v_norm) < tol:
        return np.zeros(3,)
    else:
        tau_hat = np.arctan2(v_norm, w) * v / v_norm
        return tau_hat

def Exp(tau):
    theta = np.linalg.norm(tau)
    if abs(theta) < tol:
        return np.array([1., 0., 0., 0.])
    u = tau / theta
    q = np.array([np.cos(theta / 2), np.sin(theta / 2) * u[0], np.sin(theta / 2) * u[1], np.sin(theta / 2) * u[2]])
    return q

def Log(q):
    w, x, y, z = q[0], q[1], q[2], q[3]
    v = np.array([x, y, z])
    v_norm = np.linalg.norm(v)
    if abs(v_norm) < tol:
        return np.zeros(3,)
    else:
        tau = 2. * np.arctan2(v_norm, w) * v / v_norm
        return tau

def plus_right(q, tau):
    return group_composition(q, Exp(tau))

def plus_left(q, tau):
    return group_composition(Exp(tau), q)

def minus_right(q1, q2):
    return Log(group_composition(group_inverse(q2), q1))

def minus_left(q1, q2):
    return Log(group_composition(q1, group_inverse(q2)))

def adjoint(q):
    return Rq_mat(q)

def jacobian_inverse(q):
    return -Rq_mat(q)

def jacobian_composition_1(q1, q2):
    return Rq_mat(group_inverse(q2))

def jacobian_composition_2(q1, q2):
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

def jacobian_plus_right_1(q, tau):
    Exp_tau = Exp(tau)
    return Rq_mat(group_inverse(Exp_tau))

def jacobian_plus_right_2(q, tau):
    return jacobian_right(tau)

def jacobian_minus_right_1(q1, q2):
    return jacobian_right_inverse(minus_right(q1, q2))

def jacobian_minus_right_2(q1, q2):
    return -jacobian_left_inverse(minus_right(q1, q2))

def jacobian_rotation_action_1(q, x):
    return -Rq_mat(q) @ vec_hat(x)

def jacobian_rotation_action_2(q, x):
    return Rq_mat(q)

def Rq_mat(q):
    w, x, y, z = q[0], q[1], q[2], q[3]
    Rq = np.array([[w**2 + x**2 - y**2 - z**2, 2*(x*y - w*z), 2*(x*z + w*y)],
                  [2*(x*y + w*z), w**2 - x**2 + y**2 - z**2, 2*(y*z - w*x)],
                  [2*(x*z - w*y), 2*(y*z + w*x), w**2 - x**2 - y**2 + z**2]])
    return Rq

def vec_hat(v):
    return np.block([[0., -v[2], v[1]], [v[2], 0., -v[0]], [-v[1], v[0], 0.]])
