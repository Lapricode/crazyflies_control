import numpy as np
from visualize_crazyfly import quadrotor_visualize
from lie_theory_lib import SO3_rotation as SO3, S3_rotation as S3


"""
Quadrotor system:
state:      x = [rw, qwb, vb, omegab] \in R^(13x1)
            where   rw \in R^(3x1) is the position in the world frame
                    qwb \in R^(4x1) is the orientation quaternion of the body frame w.r.t. the world frame
                    vb \in R^(3x1) is the linear velocity in the body frame
                    omegab \in R^(3x1) is the angular velocity in the body frame
control:    u = [u1, u2, u3, u4] \in R^(4x1)
            where u_i is the angular velocity of the i-th motor in rad/s

note:   the state x comes with the quaternion qwb of the rotation matrix Rwb,
        but we use the rotation matrix Rwb directly for the dynamics and the jacobians calculations
        Rwb \in R^(3x3) is the rotation matrix of the body frame w.r.t. the world frame
        Rwb \in R^(3x3) has dimension 3
"""


g = 9.81  # acceleration due to gravity in m/s^2
l = 0.045  # length of the quadrotor arm in m
m = 0.032  # mass of the quadrotor in kg
mb = 0.020  # mass of the quadrotor's body in kg
mr = (m - mb) / 4.  # mass of each quadrotor's motor in kg
Ixx = 2.*mr*l**2
Iyy = 2.*mr*l**2
Izz = 4.*mr*l**2
Ixy = 0.0
Iyz = 0.0
Ixz = 0.0
I = np.array([[Ixx, Ixy, Ixz], [Ixy, Iyy, Iyz], [Ixz, Iyz, Izz]])  # inertia matrix of the quadrotor in kg*m^2
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
    Rwb_dot = Rwb @ SO3.hat(omegab)  # derivative of the rotation matrix of the body frame w.r.t. the world frame
    F_b = Rwb.T @ np.array([0, 0, -m * g]) + np.array([0, 0, Kf * np.sum(u)])  # force in the body frame
    T_b = np.array([l * Kf * (u[0] -u[2]), l * Kf * (u[3] - u[1]), Kt * (u[0] + u[2] - u[1] - u[3])])  # torque in the body frame
    vb_dot = 1/m * F_b - np.cross(omegab, vb)  # linear acceleration in the body frame
    omegab_dot = I_inv @ (T_b - np.cross(omegab, I @ omegab))  # angular acceleration in the body frame
    return rw_dot, Rwb_dot, vb_dot, omegab_dot

def quadrotor_dynamics_dx(x, u):
    A = np.zeros((N, N))
    rw = x[:3]
    qwb = x[3:7]
    Rwb = S3.Rq_mat(qwb)
    vb = x[7:10]
    omegab = x[10:13]
    vb_hat = SO3.vec_hat(vb)
    omegab_hat = SO3.vec_hat(omegab)
    A[0:3, 3:6] = -Rwb @ vb_hat
    A[0:3, 6:9] = Rwb
    # A[3:6, 3:6] = np.eye(3)
    # A[3:6, 9:12] = np.eye(3)
    A[6:9, 3:6] = Rwb.T @ SO3.vec_hat(np.array([0., 0., -g])) @ Rwb
    A[6:9, 6:9] = -omegab_hat
    A[6:9, 9:12] = vb_hat
    A[9:12, 9:12] = -I_inv @ (omegab_hat @ I - SO3.vec_hat(I @ omegab))
    return A

def quadrotor_dynamics_du(x, u):
    B = np.zeros((N, M))
    B[8, :] = np.array([Kf / m, Kf / m, Kf / m, Kf / m])
    B[9:12, :] = I_inv @ np.array([[l * Kf, 0., -l*Kf, 0.], \
                                    [0., -l*Kf, 0., l*Kf], \
                                    [Kt, -Kt, Kt, -Kt]])
    # B[9:12, 0] = I_inv @ np.array([l * Kf, 0., Kt])
    # B[9:12, 1] = I_inv @ np.array([0., -l * Kf, -Kt])
    # B[9:12, 2] = I_inv @ np.array([-l * Kf, 0., Kt])
    # B[9:12, 3] = I_inv @ np.array([0., l * Kf, -Kt])
    return B

def euler(xk, uk, dynamics, dt):
    rw = xk[:3]
    qwb = xk[3:7]
    # Rwb = S3.Rq_mat(qwb)
    vb = xk[7:10]
    omegab = xk[10:13]
    rw_dot, Rwb_dot, vb_dot, omegab_dot = dynamics(xk, uk)
    rw += rw_dot * dt
    qwb = S3.plus_right(qwb, omegab * dt)
    # Rwb = SO3.plus_right(Rwb, omegab * dt)
    vb += vb_dot * dt
    omegab += omegab_dot * dt
    x = np.concatenate([rw, qwb, vb, omegab])
    return x

def linearize_euler(x_bar, u_bar, dt):
    rw = x_bar[:3]
    qwb = x_bar[3:7]
    Rwb = S3.Rq_mat(qwb)
    vb = x_bar[7:10]
    omegab = x_bar[10:13]
    vb_hat = SO3.vec_hat(vb)
    omegab_hat = SO3.vec_hat(omegab)

    A = np.zeros((N, N))
    A[0:3, 0:3] = np.eye(3)
    A[0:3, 3:6] = -Rwb @ vb_hat * dt
    A[0:3, 6:9] = Rwb * dt
    A[3:6, 3:6] = SO3.jacobian_plus_right_1(Rwb, omegab * dt)
    A[3:6, 9:12] = SO3.jacobian_plus_right_2(Rwb, omegab * dt) * dt
    A[6:9, 3:6] = Rwb.T @ SO3.vec_hat(np.array([0., 0., -g])) @ Rwb * dt
    A[6:9, 6:9] = np.eye(3) - omegab_hat * dt
    A[6:9, 9:12] = vb_hat * dt
    A[9:12, 9:12] = np.eye(3) - I_inv @ (omegab_hat @ I - SO3.vec_hat(I @ omegab)) * dt

    B = np.zeros((N, M))
    B[8, :] = np.array([Kf / m, Kf / m, Kf / m, Kf / m]) * dt
    B[9:12, :] = I_inv @ np.array([[l*Kf, 0., -l*Kf, 0.], \
                                    [0., -l*Kf, 0., l*Kf], \
                                    [Kt, -Kt, Kt, -Kt]]) * dt
    return A, B

def runge_kutta_4th_order(xk, uk, dynamics, dt):
    f1 = dynamics(xk, uk)
    f2 = dynamics(xk + f1 * dt / 2., uk)
    f3 = dynamics(xk + f2 * dt / 2., uk)
    f4 = dynamics(xk + f3 * dt, uk)
    x = xk + dt / 6. * (f1 + 2. * f2 + 2. * f3 + f4)
    return x

def linearize_runge_kutta(x_bar, u_bar, dynamics, dt):
    A = np.zeros((N, N))
    B = np.zeros((N, M))
    return A, B

def lqr(A, B, Q, Qf, R, Knum):
    Kc = np.zeros((M, N))
    Pc = Qf
    for k in range(Knum-2, -1, -1):
        tmp_1 = R + B.T @ Pc @ B
        tmp_2 = B.T @ Pc @ A
        Kc = np.linalg.solve(tmp_1, tmp_2)
        # tmp = A - B @ Kc
        # Pc = Q + Kc.T @ R @ Kc + tmp.T @ Pc @ tmp
        Pc = Q + A.T @ Pc @ (A - B @ Kc)
    return Kc  # return the gain matrix Kinf

def tvlqr(xk, uk, Q, Qf, R, Knum):
    Kc_list = [np.zeros((M, N)) for _ in range(Knum)]
    Pc = Qf
    for k in range(Knum - 2, -1, -1):
        Ak, Bk = linearize_euler(xk[k], uk[k], dt)
        tmp_1 = R + Bk.T @ Pc @ Bk
        tmp_2 = Bk.T @ Pc @ Ak
        Kc = np.linalg.solve(tmp_1, tmp_2)
        # tmp = A - B @ Kc
        # Pc = Q + Kc.T @ R @ Kc + tmp.T @ Pc @ tmp
        Pc = Q + Ak.T @ Pc @ (Ak - Bk @ Kc)
        Kc_list[k] = Kc
    return Kc_list  # return the gain matrices Kc for each time step

def compute_state_right_minus(x_1, x_2):
    rw_1 = x_1[:3]
    qwb_1 = x_1[3:7]
    Rwb_1 = S3.Rq_mat(qwb_1)
    vb_1 = x_1[7:10]
    omegab_1 = x_1[10:13]
    rw_2 = x_2[:3]
    qwb_2 = x_2[3:7]
    Rwb_2 = S3.Rq_mat(qwb_2)
    vb_2 = x_2[7:10]
    omegab_2 = x_2[10:13]
    rw_error = rw_1 - rw_2
    Rwb_error = SO3.minus_right(Rwb_1, Rwb_2)
    vb_error = vb_1 - vb_2
    omegab_error = omegab_1 - omegab_2
    x_error = np.concatenate([rw_error, Rwb_error, vb_error, omegab_error])
    return x_error  # \in R^(12x1)

def compute_state_right_plus(x_1, x_2):
    rw_1 = x_1[:3]
    qwb_1 = x_1[3:7]
    vb_1 = x_1[7:10]
    omegab_1 = x_1[10:13]
    rw_2 = x_2[:3]
    qwb_2 = x_2[3:7]
    vb_2 = x_2[7:10]
    omegab_2 = x_2[10:13]
    rw_sum = rw_1 + rw_2
    qwb_sum = S3.plus_right(qwb_1, qwb_2)
    vb_sum = vb_1 + vb_2
    omegab_sum = omegab_1 + omegab_2
    x_sum = np.concatenate([rw_sum, qwb_sum, vb_sum, omegab_sum])
    return x_sum  # \in R^(13x1)

def run_quadrotor_regulate_configuration(x0, x_ref, u_bar, u_lim, Kinf, dt, tf):
    x = np.copy(x0)
    positions = []
    orientations = []
    for k in range(round(tf / dt)):
        rw = x[:3]
        Rwb = S3.Rq_mat(x[3:7])
        positions.append(rw)
        orientations.append(Rwb)
        u = u_bar - Kinf @ (compute_state_right_minus(x, x_ref))
        # u = 1.0 * np.pi/5. * np.array([1, 1, 1, 1])
        u = np.clip(u, -u_lim, u_lim)
        x = euler(x, u, quadrotor_dynamics, dt)
    print(f"Final error: {compute_state_right_minus(x, x_ref)}")
    return positions, orientations

def run_quadrotor_regulate_trajectory(x0, xk, u_bar, u_lim, Kc, dt, tf):
    x = np.copy(x0)
    positions = []
    orientations = []
    for k in range(round(tf / dt)):
        rw = x[:3]
        Rwb = S3.Rq_mat(x[3:7])
        positions.append(rw)
        orientations.append(Rwb)
        u = u_bar - Kc[k] @ (compute_state_right_minus(x, xk[k]))
        u = np.clip(u, -u_lim, u_lim)
        x = euler(x, u, quadrotor_dynamics, dt)
    print(f"Final error: {compute_state_right_minus(x, xk[k])}")
    return positions, orientations


dt = 1e-2
tf = 50
Q = 10. * np.eye(N)
Qf = 10. * np.eye(N)
R = 1. * np.eye(M)
x0 = np.array([0.0, 0.0, 0.0, \
                1.0, 0.0, 0.0, 0.0, \
                0.0, 0.0, 0.0, \
                0.0, 0.0, 0.0])
u_ref = m*g/4 / Kf * np.ones(4)  # hovering control input
u_lim = 10. * (2. * np.pi) * np.ones(4)

# r_ref = np.array([1.0, 2.0, 0.7])
# phi_ref = np.pi / 3.0
# q_ref = np.array([np.cos(phi_ref / 2.), 0.0, 0.0, np.sin(phi_ref / 2.)])
# x_ref = np.block([r_ref, \
#                   q_ref, \
#                   0.0, 0.0, 0.0, \
#                   0.0, 0.0, 0.0])
# u_bar = np.copy(u_ref)
# x_bar = np.copy(x_ref)
# A, B = linearize_euler(x_bar, u_bar, dt)
# Kinf = lqr(A, B, Q, Qf, R, round(tf / dt))
# print(f"Kinf: {np.round(Kinf, 5)}")
# positions, orientations = run_quadrotor_regulate_configuration(x0, x_ref, u_bar, u_lim, Kinf, dt, tf)

total_time_vec = np.arange(0, tf, dt)
tt = 1/10 * tf
x_initial_traj = np.array([1.0, 2.0, 0.7, \
                            1.0, 0.0, 0.0, 0.0, \
                            0.0, 0.0, 0.0, \
                            0.0, 0.0, 0.0])
r = 1.0
freq = 0.1
x_center = x_initial_traj[0] - r
y_center = x_initial_traj[1]
h = x_initial_traj[2]
xk_traj = [np.array([x_center + r * np.cos(2 * np.pi * freq * t), y_center + r * np.sin(2 * np.pi * freq * t), h, \
                    1.0, 0.0, 0.0, 0.0, \
                    0.0, 0.0, 0.0, \
                    0.0, 0.0, 0.0]) for t in np.arange(0, tf - tt, dt)]
uk_traj = len(total_time_vec) * [u_ref]
A, B = linearize_euler(x_initial_traj, u_ref, dt)
Kinf = lqr(A, B, Q, Qf, R, round(tt / dt))
positions_1, orientations_1 = run_quadrotor_regulate_configuration(x0, x_initial_traj, u_ref, u_lim, Kinf, dt, tt)
Kc = tvlqr(xk_traj, uk_traj, Q, Qf, R, round((tf - tt) / dt))
x0_tt = np.block([np.array(positions_1[-1]), \
                    1.0, 0.0, 0.0, 0.0, \
                    0.0, 0.0, 0.0, \
                    0.0, 0.0, 0.0])
positions_2, orientations_2 = run_quadrotor_regulate_trajectory(x0_tt, xk_traj, u_ref, u_lim, Kc, dt, tf - tt)
positions = positions_1 + positions_2
orientations = orientations_1 + orientations_2

quadrotor_visualize(positions, orientations, 0.01, 10, 0.5, cam_onboard = True)
