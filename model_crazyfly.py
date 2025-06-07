import numpy as np
from visualize_crazyflie import quadrotor_visualize
from lie_theory_lib import SO3_rotation as SO3, S3_rotation as S3


l = 0.05  # length of the quadrotor arm in m
g = 9.81  # acceleration due to gravity in m/s^2
m = 0.05  # mass of the quadrotor in kg
I = np.diag([1e-3, 1e-3, 1e-3])  # inertia matrix of the quadrotor in kg*m^2
I_inv = np.linalg.inv(I)  # inverse of the inertia matrix
Kf = 1e-2  # thrust coefficient in N/(rad/s)
Kt = 1e-2  # torque coefficient in N*m/(rad/s)

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
    v_b_dot = 1/m * F_b - np.cross(omegab, vb)  # linear acceleration in the body frame
    omega_b_dot = I_inv @ (T_b - np.cross(omegab, I @ omegab))  # angular acceleration in the body frame
    return rw_dot, v_b_dot, omega_b_dot

def quadrotor_dynamics_dx(x, u):
    pass

def quadrotor_dynamics_du(x, u):
    pass

u = 1.0 * np.pi/5. * np.array([1, 1, 1, 1])
x0 = np.array([0.0, 0.0, 0.0, \
                1.0, 0.0, 0.0, 0.0, \
                0.0, 0.0, 0.0, \
                0.0, 0.0, 0.0])
dt = 1e-2
tf = 2
N = int(tf / dt) + 1
positions = []
orientations = []
x = np.copy(x0)
for k in range(N):
    rw = x[:3]
    qwb = x[3:7]
    Rwb = S3.Rq_mat(qwb)
    vb = x[7:10]
    omegab = x[10:13]
    positions.append(rw)
    orientations.append(Rwb)
    
    rw_dot, vb_dot, omegab_dot = quadrotor_dynamics(x, u)
    rw += rw_dot * dt
    Rwb = SO3.plus_right(Rwb, x[10:13] * dt)
    qwb = S3.plus_right(qwb, x[10:13] * dt)
    vb += vb_dot * dt
    omegab += omegab_dot * dt
    x = np.concatenate([rw, qwb, vb, omegab])
    

quadrotor_visualize(positions, orientations, dt, 1, l)
