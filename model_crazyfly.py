import numpy as np
from visualize_crazyfly import quadrotor_visualize
from lie_theory_lib import SO3_rotation as SO3, S3_rotation as S3


"""
Quadrotor system:
state:      x = [rw, Rwb, vb, omegab] \in R^(12x1)
            where   rw \in R^(3x1) is the position in the world frame
                    Rwb \in R^(3x3) is the rotation matrix of the body frame w.r.t. the world frame
                    Rwb \in R^(3x3) has dimension 3
                    vb \in R^(3x1) is the linear velocity in the body frame
                    omegab \in R^(3x1) is the angular velocity in the body frame
control:    u = [u1, u2, u3, u4] \in R^(4x1)
            where u_i is the angular velocity of the i-th motor in rad/s

note: the state x comes with the quaternion qwb of the rotation matrix Rwb,
        but we use the rotation matrix Rwb directly for the dynamics and the jacobians calculations
"""


l = 0.05  # length of the quadrotor arm in m
g = 9.81  # acceleration due to gravity in m/s^2
m = 0.05  # mass of the quadrotor in kg
I = np.diag([1e-3, 1e-3, 1e-3])  # inertia matrix of the quadrotor in kg*m^2
I_inv = np.linalg.inv(I)  # inverse of the inertia matrix
Kf = 1e-2  # thrust coefficient in N/(rad/s)
Kt = 1e-2  # torque coefficient in N*m/(rad/s)
N = 12  # number of states in the quadrotor dynamics
M = 4  # number of inputs in the quadrotor dynamics

def quadrotor_dynamics(x, u):
    x = x.reshape(-1)
    u = u.reshape(-1)
    rw = x[:3]  # position in the world frame
    qwb = x[3:7]  # orientation quaternion of the body frame w.r.t. the world frame
    vb = x[7:10]  # linear velocity in the body frame
    omegab = x[10:13]  # angular velocity in the body frame
    Rwb = S3.Rq_mat(qwb)  # rotation matrix of the body frame w.r.t. the world frame
    rw_dot = Rwb @ vb  # linear velocity in the world frame
    # Rwb_dot = Rwb @ SO3.hat(omegab)  # derivative of the rotation matrix of the body frame w.r.t. the world frame
    F_b = Rwb.T @ np.array([0, 0, -m * g]) + np.array([0, 0, Kf * np.sum(u)])  # force in the body frame
    T_b = np.array([l * Kf * (u[0] -u[2]), l * Kf * (u[3] - u[1]), Kt * (u[0] + u[2] - u[1] - u[3])])  # torque in the body frame
    vb_dot = 1/m * F_b - np.cross(omegab, vb)  # linear acceleration in the body frame
    omegab_dot = I_inv @ (T_b - np.cross(omegab, I @ omegab))  # angular acceleration in the body frame
    return rw_dot, vb_dot, omegab_dot

def quadrotor_dynamics_dx(x, u):
    return

def quadrotor_dynamics_du(x, u):
    B = np.zeros((N, M))
    B[8, :] = np.array([Kf / m, Kf / m, Kf / m, Kf / m])
    B[9:12, 0] = I_inv @ np.array([l * Kf, 0., Kt])
    B[9:12, 1] = I_inv @ np.array([0., -l * Kf, -Kt])
    B[9:12, 2] = I_inv @ np.array([-l * Kf, 0., Kt])
    B[9:12, 3] = I_inv @ np.array([0., l * Kf, -Kt])
    return B

def linearize_quadrotor_dynamics(x_bar, u_bar, dynamics, dt):
    A = np.zeros((N, N))
    B = np.zeros((N, M))
    return A, B

def runge_kutta_4th_order(xk, uk, dynamics, dt):
    f1 = dynamics(xk, uk)
    f2 = dynamics(xk + f1 * dt / 2., uk)
    f3 = dynamics(xk + f2 * dt / 2., uk)
    f4 = dynamics(xk + f3 * dt, uk)
    x = xk + dt / 6. * (f1 + 2. * f2 + 2. * f3 + f4)
    return x

def lqr(A, B, Q, Qf, R):
    K = 100
    Kc = np.zeros((M, N))
    Pc = Qf
    for k in range(K-2, -1, -1):
        tmp_1 = R + B.T @ Pc @ B
        tmp_2 = B.T @ Pc @ A
        Kc = np.linalg.solve(tmp_1, tmp_2)
        Pc = Q + A.T @ Pc @ (A - B @ Kc)
    return Kc  # return the gain matrix Kinf

def tvlqr(xkstar_list, ukstar_list, Q, Qf, R):
    Kc = [np.zeros((M, N)) for _ in range(len(ukstar_list))]
    Pc = Qf
    for k in range(len(xkstar_list) - 2, -1, -1):
        Ak, Bk = linearize_quadrotor_dynamics(xkstar_list[k], ukstar_list[k], quadrotor_dynamics, dt)
        tmp_1 = R + Bk.T @ Pc @ Bk
        tmp_2 = Bk.T @ Pc @ Ak
        Kc = np.linalg.solve(tmp_1, tmp_2)
        Pc = Q + Ak.T @ Pc @ (Ak - Bk @ Kc)
        Kc[k] = Kc
    return Kc  # return the gain matrices Kc for each time step

def compute_state_error(x, x_ref):
    rw = x[:3]
    qwb = x[3:7]
    Rwb = S3.Rq_mat(qwb)
    vb = x[7:10]
    omegab = x[10:13]
    rw_ref = x_ref[:3]
    qwb_ref = x_ref[3:7]
    Rwb_ref = S3.Rq_mat(qwb_ref)
    vb_ref = x_ref[7:10]
    omegab_ref = x_ref[10:13]
    rw_error = rw - rw_ref
    Rwb_error = SO3.minus_right(Rwb, Rwb_ref)
    vb_error = vb - vb_ref
    omegab_error = omegab - omegab_ref
    x_error = np.concatenate([rw_error, Rwb_error, vb_error, omegab_error])
    return x_error

def run_quadrotor(x0, x_ref, u_bar, u_lim, K_inf, dt, tf):
    x = np.copy(x0)
    positions = []
    orientations = []
    for k in range(int(tf / dt)):
        rw = x[:3]
        qwb = x[3:7]
        Rwb = S3.Rq_mat(qwb)
        vb = x[7:10]
        omegab = x[10:13]
        positions.append(rw)
        orientations.append(Rwb)

        u = u_bar - K_inf @ (compute_state_error(x, x_ref))
        # u = 1.0 * np.pi/5. * np.array([1, 1, 1, 1])
        u = np.clip(u, -u_lim, u_lim)
        rw_dot, vb_dot, omegab_dot = quadrotor_dynamics(x, u)
        rw += rw_dot * dt
        Rwb = SO3.plus_right(Rwb, x[10:13] * dt)
        qwb = S3.plus_right(qwb, x[10:13] * dt)
        vb += vb_dot * dt
        omegab += omegab_dot * dt
        x = np.concatenate([rw, qwb, vb, omegab])

    return positions, orientations


u_lim = 10. * (2. * np.pi) * np.ones(4)
u_bar = m*g/4 / Kf * np.ones(4)  # hovering control input
x0 = np.array([0.0, 0.0, 0.0, \
                1.0, 0.0, 0.0, 0.0, \
                0.0, 0.0, 0.0, \
                0.0, 0.0, 0.0])
x_ref = np.array([0.0, 0.0, 1.0, \
                  1.0, 0.0, 0.0, 0.0, \
                  0.0, 0.0, 0.0, \
                  0.0, 0.0, 0.0])
dt = 1e-2
tf = 2
K_inf = np.zeros((4, 13))

positions, orientations = run_quadrotor(x0, x_ref, u_bar, u_lim, K_inf, dt, tf)
quadrotor_visualize(positions, orientations, dt, 1, l)
