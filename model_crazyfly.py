import numpy as np
from visualize_crazyfly import quadrotor_visualize
import SO3_rotation as SO3
import S3_rotation as S3


"""
Quadrotor system:
state:      x = [rw, qwb, vb, omegab] \in R^(13x1)
            where   rw \in R^(3x1) is the position in the world frame
                    qwb \in R^(4x1) is the orientation quaternion of the body frame w.r.t. the world frame
                    vb \in R^(3x1) is the linear velocity in the body frame
                    omegab \in R^(3x1) is the angular velocity in the body frame
control:    u = [u1, u2, u3, u4] \in R^(4x1)
            where u_i is the angular speed of the i-th motor in rad/s
rotors:     Fi = Kf * ui^2
            Ti = Kt * ui^2
            where   Fi is the thrust force produced by the ith rotor
                    Ti is the torque produced by the ith rotor
            the motors are numbered in a clockwise manner, with motor 4 being in xy direction
            the rotors 1, 3 rotate counter-clockwise, and the rotors 2, 4 rotate clockwise
            the motor arms form right angles (90 degrees) with each other
note:   the state x comes with the quaternion qwb of the rotation matrix Rwb,
        but we also use the rotation matrix Rwb directly for the dynamics and the jacobians calculations
        Rwb \in R^(3x3) is the rotation matrix of the body frame w.r.t. the world frame
        Rwb \in R^(3x3) has dimension 3, this is the dimension of its Lie algebra vector
"""


g = 9.81  # acceleration due to gravity in m/s^2
l = 0.046  # length of the quadrotor arm in m
m = 0.033  # mass of the quadrotor in kg
# mb = 0.020  # mass of the quadrotor's body in kg
# mr = (m - mb) / 4.  # mass of each quadrotor's motor in kg
# Ixx = 2.*mr*l**2
# Iyy = 2.*mr*l**2
# Izz = 4.*mr*l**2
# Ixy = 0.0
# Iyz = 0.0
# Ixz = 0.0
# I = np.array([[Ixx, Ixy, Ixz], [Ixy, Iyy, Iyz], [Ixz, Iyz, Izz]])  # inertia matrix of the quadrotor in kg*m^2
I = np.array([[16.6e-6, 0.83e-6, 0.72e-6], 
                [0.83e-6, 16.6e-6, 1.8e-6],
                [0.72e-6, 1.8e-6, 29.3e-6]])
I_inv = np.linalg.inv(I)  # inverse of the inertia matrix
body_yaw0 = -3/4 * np.pi  # assuming body_yaw0 is for the motor 1 at positive y direction, motor 2 at positive x direction and clockwise motor numbers
Kf = 2.25e-8  # thrust coefficient in N/(rad/s)^2
Kt = 1.34e-10  # torque coefficient in N*m/(rad/s)^2
u_max = 25000. * (2. * np.pi / 60) * np.ones(4)  # the control input limits in rad/sec
N = 12  # number of states in the quadrotor dynamics
M = 4  # number of inputs in the quadrotor dynamics

def quadrotor_dynamics(x, u):
    rw = x[:3]  # position in the world frame
    qwb = x[3:7]  # orientation quaternion of the body frame w.r.t. the world frame
    vb = x[7:10]  # linear velocity in the body frame
    omegab = x[10:13]  # angular velocity in the body frame
    Rwb = S3.Rq_mat(qwb)  # rotation matrix of the body frame w.r.t. the world frame
    rw_dot = Rwb @ vb  # linear velocity in the world frame
    Rwb_dot = Rwb @ SO3.hat(omegab)  # derivative of the rotation matrix of the body frame w.r.t. the world frame
    F_b = Rwb.T @ np.array([0, 0, -m * g]) + np.array([0, 0, Kf * np.sum(u**2)])  # force in the body frame
    T13 = l * Kf * (u[0]**2 - u[2]**2)
    T42 = l * Kf * (u[3]**2 - u[1]**2)
    T_b = np.array([T13 * np.cos(body_yaw0) - T42 * np.sin(body_yaw0), \
                    T13 * np.sin(body_yaw0) + T42 * np.cos(body_yaw0), \
                    Kt * (-u[0]**2 - u[2]**2 + u[1]**2 + u[3]**2)])  # torque in the body frame
    vb_dot = 1/m * F_b - np.cross(omegab, vb)  # linear acceleration in the body frame
    omegab_dot = I_inv @ (T_b - np.cross(omegab, I @ omegab))  # angular acceleration in the body frame
    # return rw_dot, omegab, vb_dot, omegab_dot
    return np.concatenate([rw_dot, omegab, vb_dot, omegab_dot])

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
    B[8, :] = Kf/m * 2.*u
    cos_comp = l*Kf * np.cos(body_yaw0)
    sin_comp = l*Kf * np.sin(body_yaw0)
    B[9:12, :] = I_inv @ np.array([[cos_comp * 2.*u[0], sin_comp * 2.*u[1], -cos_comp * 2.*u[2], -sin_comp * 2.*u[3]], \
                                    [sin_comp * 2.*u[0], -cos_comp * 2.*u[1], -sin_comp * 2.*u[2], cos_comp * 2.*u[3]], \
                                    [-Kt * 2.*u[0], Kt * 2.*u[1], -Kt * 2.*u[2], Kt * 2.*u[3]]])
    return B

def finite_differences(f, x, u, dt, eps = 1e-5):
    A_fd = np.zeros((N, N))
    for i in range(N):
        ei = np.zeros(N)
        ei[i] = eps
        fx_plus = f(compute_state_right_plus(x, ei), u, dt)
        fx_minus = f(compute_state_right_plus(x, -ei), u, dt)
        A_fd[:, i] = compute_state_right_minus(fx_plus, fx_minus) / (2*eps)
        # fx_minus = f(x, u, dt)
        # A_fd[:, i] = compute_state_right_minus(fx_plus, fx_minus) / eps
    B_fd = np.zeros((N, M))
    for j in range(M):
        ej = np.zeros(M)
        ej[j] = eps
        fu_plus = f(x, u + ej, dt)
        fu_minus = f(x, u - ej, dt)
        B_fd[:, j] = compute_state_right_minus(fu_plus, fu_minus) / (2*eps)
        # fu_minus = f(x, u, dt)
        # B_fd[:, j] = compute_state_right_minus(fu_plus, fu_minus) / eps
    return A_fd, B_fd

def euler(xk, uk, dynamics, dt):
    # rw = xk[:3]
    # qwb = xk[3:7]
    # Rwb = S3.Rq_mat(qwb)
    # vb = xk[7:10]
    # omegab = xk[10:13]
    # dyn = dynamics(xk, uk)
    # rw_dot, omegab, vb_dot, omegab_dot = dyn[0:3], dyn[3:6], dyn[6:9], dyn[9:12]
    # rw = np.copy(rw + rw_dot * dt)
    # vb = np.copy(vb + vb_dot * dt)
    # omegab = np.copy(omegab + omegab_dot * dt)
    # # qwb = S3.plus_right(qwb, omegab * dt)
    # # x = np.concatenate([rw, qwb, vb, omegab])
    # Rwb = SO3.plus_right(Rwb, omegab * dt)
    # x = np.concatenate([rw, SO3.qR_quat(Rwb), vb, omegab])
    x = compute_state_right_plus(xk, dynamics(xk, uk) * dt)
    return x

# wrap euler so its signature is (x, u, dt) → next x
def euler_wrap(x, u, dt):
    return euler(x, u, quadrotor_dynamics, dt)

def linearize_euler(x_bar, u_bar, dt):
    rw = x_bar[:3]
    qwb = x_bar[3:7]
    Rwb = S3.Rq_mat(qwb)
    vb = x_bar[7:10]
    omegab = x_bar[10:13]
    vb_hat = SO3.vec_hat(vb)
    omegab_hat = SO3.vec_hat(omegab)
    # A Jacobian for the state is an N x N matrix
    A = np.zeros((N, N))
    A[0:3, 0:3] = np.eye(3)
    A[0:3, 3:6] = -Rwb @ vb_hat * dt
    A[0:3, 6:9] = Rwb * dt
    A[3:6, 3:6] = SO3.jacobian_plus_right_1(Rwb, omegab * dt)
    A[3:6, 9:12] = SO3.jacobian_plus_right_2(Rwb, omegab * dt) * dt
    # A[3:6, 3:6] = S3.jacobian_plus_right_1(qwb, omegab * dt)
    # A[3:6, 9:12] = S3.jacobian_plus_right_2(qwb, omegab * dt) * dt
    A[6:9, 3:6] = Rwb.T @ SO3.vec_hat(np.array([0., 0., -g])) @ Rwb * dt
    A[6:9, 6:9] = np.eye(3) - omegab_hat * dt
    A[6:9, 9:12] = vb_hat * dt
    A[9:12, 9:12] = np.eye(3) - I_inv @ (omegab_hat @ I - SO3.vec_hat(I @ omegab)) * dt
    # B Jacobian for the control input is an N x M matrix
    B = np.zeros((N, M))
    B[8, :] = Kf/m * 2.*u_bar * dt
    cos_comp = l * Kf * np.cos(body_yaw0)
    sin_comp = l * Kf * np.sin(body_yaw0)
    B[9:12, :] = I_inv @ np.array([[cos_comp * 2.*u_bar[0], sin_comp * 2.*u_bar[1], -cos_comp * 2.*u_bar[2], -sin_comp * 2.*u_bar[3]], \
                                    [sin_comp * 2.*u_bar[0], -cos_comp * 2.*u_bar[1], -sin_comp * 2.*u_bar[2], cos_comp * 2.*u_bar[3]], \
                                    [-Kt * 2.*u_bar[0], Kt * 2.*u_bar[1], -Kt * 2.*u_bar[2], Kt * 2.*u_bar[3]]]) * dt
    return A, B

def runge_kutta_4th_order(xk, uk, dynamics, dt):
    f1 = dynamics(xk, uk)
    f2 = dynamics(compute_state_right_plus(xk, f1 * dt / 2), uk)
    f3 = dynamics(compute_state_right_plus(xk, f2 * dt / 2.), uk)
    f4 = dynamics(compute_state_right_plus(xk, f3 * dt), uk)
    x = compute_state_right_plus(xk, (f1 + 2. * f2 + 2. * f3 + f4) * dt / 6.)
    return x

# wrap runge_kutta_4th_order so its signature is (x, u, dt) → next x
def rk4_wrap(x, u, dt):
    return runge_kutta_4th_order(x, u, quadrotor_dynamics, dt)

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
    # x_1 and x_2 are both \in R^(13x1)
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
    vb_error = vb_1 - vb_2
    omegab_error = omegab_1 - omegab_2
    # qwb_error = S3.minus_right(qwb_1, qwb_2)
    # x_error = np.concatenate([rw_error, qwb_error, vb_error, omegab_error])
    Rwb_error = SO3.minus_right(Rwb_1, Rwb_2)
    x_error = np.concatenate([rw_error, Rwb_error, vb_error, omegab_error])
    return x_error  # \in R^(12x1)

def compute_state_right_plus(x, dx):
    # x is \in R^(13x1) and dx is \in R^(12x1)
    rw = x[:3]
    qwb = x[3:7]
    Rwb = S3.Rq_mat(qwb)
    vb = x[7:10]
    omegab = x[10:13]
    drw = dx[:3]
    dqwb = dx[3:6]
    dRwb = dx[3:6]
    dvb = dx[6:9]
    domegab = dx[9:12]
    rw_sum = rw + drw
    vb_sum = vb + dvb
    omegab_sum = omegab + domegab
    # qwb_sum = S3.plus_right(qwb, dqwb)
    # x_sum = np.concatenate([rw_sum, qwb_sum, vb_sum, omegab_sum])
    Rwb_sum = SO3.plus_right(Rwb, dRwb)
    x_sum = np.concatenate([rw_sum, SO3.qR_quat(Rwb_sum), vb_sum, omegab_sum])
    return x_sum  # \in R^(13x1)

def run_quadrotor_regulate_configuration(x0, x_ref, u_bar, u_max, Kinf, dt, tf):
    x = np.copy(x0)
    positions = []
    orientations = []
    control_inputs = []
    for k in range(round(tf / dt)):
        rw = x[:3]
        Rwb = S3.Rq_mat(x[3:7])
        positions.append(rw)
        orientations.append(Rwb)
        u = u_bar - Kinf @ (compute_state_right_minus(x, x_ref))
        # u = 1.0 * np.pi/5. * np.array([1, 1, 1, 1])
        u = np.clip(u, 0.0, u_max)
        control_inputs.append(u)
        x = euler(x, u, quadrotor_dynamics, dt)
        # x = runge_kutta_4th_order(x, u, quadrotor_dynamics, dt)
    print(f"Final error: {compute_state_right_minus(x, x_ref)}")
    return positions, orientations, control_inputs

def run_quadrotor_track_trajectory(x0, xk, u_bar, u_max, Kc, dt, tf):
    x = np.copy(x0)
    positions = []
    orientations = []
    control_inputs = []
    for k in range(round(tf / dt)):
        rw = x[:3]
        Rwb = S3.Rq_mat(x[3:7])
        positions.append(rw)
        orientations.append(Rwb)
        # noise = np.random.rand(N+1) / 10.
        # noise[7:N+1] = np.zeros((N-6,))
        # x_noisy = x + noise
        # x_noisy[3:7] = x_noisy[3:7] / np.linalg.norm(x_noisy)
        u = u_bar - Kc[k] @ (compute_state_right_minus(x, xk[k]))
        u = np.clip(u, 0.0, u_max)
        control_inputs.append(u)
        x = euler(x, u, quadrotor_dynamics, dt)
        # x = runge_kutta_4th_order(x, u, quadrotor_dynamics, dt)
    return positions, orientations, control_inputs

def save_Kinf_mat(Kinf, file):
    lines = []
    for row in Kinf:
        formatted_row = ", ".join(f"{val:.8e}f" for val in row)
        lines.append(f"{{{formatted_row}}}")
    cpp_struct_str = "static const float Kinf[4][12] = {\n"
    cpp_struct_str += ",\n".join(f"    {line}" for line in lines)
    cpp_struct_str += "};\n"
    with open(file, "w") as f:
        f.write(cpp_struct_str)
    print(f"Kinf matrix saved to \"{file}\" in C-style struct format.")

def rpy_to_quat(rpy):  # rpy is an array: [roll, pitch, yaw], in radians
    roll, pitch, yaw = rpy[0], rpy[1], rpy[2]
    cr = np.cos(0.5 * roll)
    sr = np.sin(0.5 * roll)
    cp = np.cos(0.5 * pitch)
    sp = np.sin(0.5 * pitch)
    cy = np.cos(0.5 * yaw)
    sy = np.sin(0.5 * yaw)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return np.array([w, x, y, z])


dt = 1e-2
yaw_bar = 0.0
q_bar = rpy_to_quat(np.array([0.0, 0.0, yaw_bar]))
x_bar = np.block([0.0, 0.0, 0.0, \
                    q_bar, \
                    0.0, 0.0, 0.0, \
                    0.0, 0.0, 0.0])
u_bar = np.sqrt(m*g/4 / Kf) * np.ones(4)
# rpy_bar = np.array([0.5, 0.7, 1.0])
# q_bar = rpy_to_quat(rpy_bar)
# x_bar = 5. * np.random.randn(13,)
# x_bar[3:7] = q_bar
# u_bar = 1000. * np.abs(np.random.randn(4,))
A_num, B_num = finite_differences(euler_wrap, x_bar, u_bar, dt, 1e-5)
A_an, B_an  = linearize_euler(x_bar, u_bar, dt)
# dec_digits_shown = 3
# print("A_an: ", np.round(A_an, dec_digits_shown))
# print("A_num: ", np.round(A_num, dec_digits_shown))
# print(np.round(A_an - A_num, dec_digits_shown))
# print("B_an: ", np.round(B_an, dec_digits_shown))
# print("B_num: ", np.round(B_num, dec_digits_shown))
# print(np.round(B_an - B_num, dec_digits_shown))
print("x_bar =", x_bar)
print("u_bar =", u_bar)
print("‖A_num − A_an‖_∞ =", np.max(np.abs(A_num - A_an)))
print("‖B_num − B_an‖_∞ =", np.max(np.abs(B_num - B_an)))

# setup the cost matrices of the LQR controller, and the initial state
Q = np.eye(N)
Q[0:3, 0:3] = 10000. * np.eye(3)
Q[3:6, 3:6] = 10000. * np.eye(3)
Q[6:9, 6:9] = 1000. * np.eye(3)
Q[9:12, 9:12] = 5000. * np.eye(3)
Qf = 10000. * np.eye(N)
R = 0.001 * np.eye(M)
r0 = np.array([0.0, 0.0, 0.0])
rpy0 = np.array([0.0, 0.0, 0.0])
q0 = rpy_to_quat(rpy0)
x0 = np.block([r0, \
                q0, \
                0.0, 0.0, 0.0, \
                0.0, 0.0, 0.0])

# regulate a certain crazyflie configuration (position and yaw orientation)
dt = 1e-2
tf = 10
r_bar = np.array([0.0, 0.0, 0.0])
yaw_bar = 0.0
rpy_bar = np.array([0.0, 0.0, yaw_bar])
q_bar = rpy_to_quat(rpy_bar)
x_bar = np.block([r_bar, \
                    q_bar, \
                    0.0, 0.0, 0.0, \
                    0.0, 0.0, 0.0])
u_bar = np.sqrt(m*g/4 / Kf) * np.ones(4)  # hovering control input
A, B = linearize_euler(x_bar, u_bar, dt)
# A, B = finite_differences(euler_wrap, x_bar, u_bar, dt, 1e-5)
# A, B = finite_differences(rk4_wrap, x_bar, u_bar, dt, 1e-5)
Kinf = lqr(A, B, Q, Qf, R, round(tf / dt))
print(f"Kinf: {np.round(Kinf, 5)}")
r_ref = np.array([1.0, 1.0, 1.0])
rpy_ref = np.array([0.0, 0.0, 0.0])
q_ref = rpy_to_quat(rpy_ref)
x_ref = np.block([r_ref, \
                    q_ref, \
                    0.0, 0.0, 0.0, \
                    0.0, 0.0, 0.0])
u_ref = np.sqrt(m*g/4 / Kf) * np.ones(4)  # hovering control input
positions, orientations, control_inputs = run_quadrotor_regulate_configuration(x0, x_ref, u_bar, u_max, Kinf, dt, tf)
save_Kinf_mat(Kinf, "Kinf.txt")
quadrotor_visualize(positions, orientations, control_inputs, u_max, 0.01, int(1/(10*dt)), 0.25, body_yaw0, cam_onboard = True)

# # track a circular trajectory
# dt = 1e-2
# tf = 10
# total_time_vec = np.arange(0, tf, dt)
# tt = 1/5 * tf
# x_initial_traj = np.array([1.0, 2.0, 0.5, \
#                             1.0, 0.0, 0.0, 0.0, \
#                             0.0, 0.0, 0.0, \
#                             0.0, 0.0, 0.0])
# r = 1.0
# freq_circle = 0.05
# freq_bodyrot = freq_circle / 2
# x_center = x_initial_traj[0] - r
# y_center = x_initial_traj[1]
# h = x_initial_traj[2]
# xk_traj = [np.array([x_center + r * np.cos(2 * np.pi * freq_circle * t), y_center + r * np.sin(2 * np.pi * freq_circle * t), h, \
#                     np.cos(((2 * np.pi * freq_bodyrot * t) % (2 * np.pi)) / 2.), 0.0, 0.0, np.sin(((2 * np.pi * freq_bodyrot * t) % (2 * np.pi)) / 2.), \
#                     0.0, 0.0, 0.0, \
#                     0.0, 0.0, 0.0]) for t in np.arange(0, tf - tt, dt)]
# uk_traj = len(total_time_vec) * [u_ref]
# A, B = linearize_euler(x_initial_traj, u_ref, dt)
# Kinf = lqr(A, B, Q, Qf, R, round(tt / dt))
# positions_1, orientations_1, control_inputs_1 = run_quadrotor_regulate_configuration(x0, x_initial_traj, u_ref, u_max, Kinf, dt, tt)
# Kc = tvlqr(xk_traj, uk_traj, Q, Qf, R, round((tf - tt) / dt))
# x0_tt = np.block([np.array(positions_1[-1]), \
#                     1.0, 0.0, 0.0, 0.0, \
#                     0.0, 0.0, 0.0, \
#                     0.0, 0.0, 0.0])
# positions_2, orientations_2, control_inputs_2 = run_quadrotor_track_trajectory(x0_tt, xk_traj, u_ref, u_max, Kc, dt, tf - tt)
# positions = positions_1 + positions_2
# orientations = orientations_1 + orientations_2
# control_inputs = control_inputs_1 + control_inputs_2
# quadrotor_visualize(positions, orientations, control_inputs, u_max, 0.01, int(1/(10*dt)), 0.5, body_yaw0, cam_onboard = False)
