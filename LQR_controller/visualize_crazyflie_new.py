import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ----------------------------
# Load data
# ----------------------------
states = np.loadtxt("animation/double_pendulum_states.csv", delimiter=",")
controls = np.loadtxt("animation/double_pendulum_controls.csv", delimiter=",")

with open("animation/double_pendulum_params.json", "r") as f:
    params = json.load(f)

q1 = states[:, 0]
q2 = states[:, 1]
dq1 = states[:, 2]
dq2 = states[:, 3]

u1 = controls[:, 0]
u2 = controls[:, 1]

l1, l2 = params["l1"], params["l2"]
dt = 1.0 / params["freq"]
states_step = 1

num_frames = len(q1)
time = np.arange(num_frames) * dt
total_time = time[-1]


# ----------------------------
# Helpers
# ----------------------------
def get_limits(arr, margin=0.1):
    mn, mx = np.min(arr), np.max(arr)
    span = mx - mn
    if span == 0:
        span = 1.0
    return mn - margin * span, mx + margin * span


q_lim = get_limits(np.concatenate([q1, q2]))
dq_lim = get_limits(np.concatenate([dq1, dq2]))
u_lim = get_limits(np.concatenate([u1, u2]))

# ----------------------------
# Figure layout
# ----------------------------
fig = plt.figure(figsize=(14, 8))

ax_vis = plt.subplot2grid((2, 2), (0, 0))
ax_traj = plt.subplot2grid((2, 2), (1, 0))
ax_pos = plt.subplot2grid((3, 2), (0, 1))
ax_vel = plt.subplot2grid((3, 2), (1, 1))
ax_ctrl = plt.subplot2grid((3, 2), (2, 1))

margin_size = 0.02
world_lim = (l1 + l2) + margin_size

# ----------------------------
# Axes setup
# ----------------------------
ax_vis.set_aspect("equal")
ax_vis.set_title("System animation")
ax_vis.set_xlabel("x (m)", loc="right")
ax_vis.set_ylabel("y (m)", loc="center")
ax_vis.set_xlim(-world_lim, world_lim)
ax_vis.set_ylim(-world_lim, world_lim)

ax_traj.set_aspect("equal")
ax_traj.set_title("System trajectories")
ax_traj.set_xlabel("x (m)", loc="right")
ax_traj.set_ylabel("y (m)", loc="center")
ax_traj.set_xlim(-world_lim, world_lim)
ax_traj.set_ylim(-world_lim, world_lim)

ax_pos.set_title("Joint Positions")
ax_pos.set_xlabel("t (sec)", loc="right")
ax_pos.set_ylabel("q (rad)", loc="center")
ax_pos.set_xlim(0, total_time)
ax_pos.set_ylim(*q_lim)

ax_vel.set_title("Joint Velocities")
ax_vel.set_xlabel("t (sec)", loc="right")
ax_vel.set_ylabel("dq/dt (rad/sec)", loc="center")
ax_vel.set_xlim(0, total_time)
ax_vel.set_ylim(*dq_lim)

ax_ctrl.set_title("Control inputs")
ax_ctrl.set_xlabel("t (sec)", loc="right")
ax_ctrl.set_ylabel("torque (Nm)", loc="center")
ax_ctrl.set_xlim(0, total_time)
ax_ctrl.set_ylim(*u_lim)

# ----------------------------
# Visual elements
# ----------------------------
ax_vis.axhline(y=0, color="brown", lw=8)

(line_l1,) = ax_vis.plot([], [], "k-", lw=3)
(line_l2,) = ax_vis.plot([], [], "k-", lw=3)

(joint1,) = ax_vis.plot([], [], "bo", markersize=10)
(joint2,) = ax_vis.plot([], [], "ro", markersize=10)
(mass,) = ax_vis.plot([], [], "ko", markersize=12)

time_text = ax_vis.text(0.05, 0.9, "", transform=ax_vis.transAxes)

(traj1_line,) = ax_traj.plot([], [], "r-", lw=2)
(traj2_line,) = ax_traj.plot([], [], "k-", lw=2)

(pos1_line,) = ax_pos.plot([], [], "b-", lw=1)
(pos2_line,) = ax_pos.plot([], [], "r-", lw=1)

(vel1_line,) = ax_vel.plot([], [], "b-", lw=1)
(vel2_line,) = ax_vel.plot([], [], "r-", lw=1)

(ctrl1_line,) = ax_ctrl.plot([], [], "b-", lw=1)
(ctrl2_line,) = ax_ctrl.plot([], [], "r-", lw=1)

cursor_pos = ax_pos.axvline(0, linestyle="--", lw=1, color="k")
cursor_vel = ax_vel.axvline(0, linestyle="--", lw=1, color="k")
cursor_ctrl = ax_ctrl.axvline(0, linestyle="--", lw=1, color="k")

# ----------------------------
# State
# ----------------------------
paused = False
frame_counter = 0

traj1_x, traj1_y = [], []
traj2_x, traj2_y = [], []


# ----------------------------
# Reset helpers
# ----------------------------
def reset_all():
    global traj1_x, traj1_y, traj2_x, traj2_y
    traj1_x, traj1_y = [], []
    traj2_x, traj2_y = [], []


def clear_all_artists():
    line_l1.set_data([], [])
    line_l2.set_data([], [])
    joint1.set_data([], [])
    joint2.set_data([], [])
    mass.set_data([], [])

    traj1_line.set_data([], [])
    traj2_line.set_data([], [])

    pos1_line.set_data([], [])
    pos2_line.set_data([], [])
    vel1_line.set_data([], [])
    vel2_line.set_data([], [])
    ctrl1_line.set_data([], [])
    ctrl2_line.set_data([], [])

    cursor_pos.set_xdata([0, 0])
    cursor_vel.set_xdata([0, 0])
    cursor_ctrl.set_xdata([0, 0])

    time_text.set_text("")


# ----------------------------
# Init
# ----------------------------
def init():
    clear_all_artists()
    joint1.set_data([0], [0])
    return (
        line_l1,
        line_l2,
        joint1,
        joint2,
        mass,
        traj1_line,
        traj2_line,
        pos1_line,
        pos2_line,
        vel1_line,
        vel2_line,
        ctrl1_line,
        ctrl2_line,
        cursor_pos,
        cursor_vel,
        cursor_ctrl,
        time_text,
    )


# ----------------------------
# Update
# ----------------------------
def update(_):
    global frame_counter, traj1_x, traj1_y, traj2_x, traj2_y

    if paused:
        return (
            line_l1,
            line_l2,
            joint1,
            joint2,
            mass,
            traj1_line,
            traj2_line,
            pos1_line,
            pos2_line,
            vel1_line,
            vel2_line,
            ctrl1_line,
            ctrl2_line,
            cursor_pos,
            cursor_vel,
            cursor_ctrl,
            time_text,
        )

    k = frame_counter * states_step

    if k >= num_frames:
        frame_counter = 0
        k = 0
        reset_all()

    # Kinematics
    x1 = l1 * np.sin(q1[k])
    y1 = -l1 * np.cos(q1[k])

    x2 = x1 + l2 * np.sin(q1[k] + q2[k])
    y2 = y1 - l2 * np.cos(q1[k] + q2[k])

    # Draw mechanism
    line_l1.set_data([0, x1], [0, y1])
    line_l2.set_data([x1, x2], [y1, y2])

    joint1.set_data([0], [0])
    joint2.set_data([x1], [y1])
    mass.set_data([x2], [y2])

    # History up to current frame
    k_hist = np.arange(0, k + 1, states_step)
    t_hist = k_hist * dt

    q1_hist = q1[k_hist]
    q2_hist = q2[k_hist]
    dq1_hist = dq1[k_hist]
    dq2_hist = dq2[k_hist]
    u1_hist = u1[k_hist]
    u2_hist = u2[k_hist]

    pos1_line.set_data(t_hist, q1_hist)
    pos2_line.set_data(t_hist, q2_hist)

    vel1_line.set_data(t_hist, dq1_hist)
    vel2_line.set_data(t_hist, dq2_hist)

    ctrl1_line.set_data(t_hist, u1_hist)
    ctrl2_line.set_data(t_hist, u2_hist)

    cursor_pos.set_xdata([k * dt, k * dt])
    cursor_vel.set_xdata([k * dt, k * dt])
    cursor_ctrl.set_xdata([k * dt, k * dt])

    time_text.set_text(f"Time: {k * dt:.2f}s")

    # Trajectories
    traj1_x = l1 * np.sin(q1[k_hist])
    traj1_y = -l1 * np.cos(q1[k_hist])

    traj2_x = traj1_x + l2 * np.sin(q1[k_hist] + q2[k_hist])
    traj2_y = traj1_y - l2 * np.cos(q1[k_hist] + q2[k_hist])

    traj1_line.set_data(traj1_x, traj1_y)
    traj2_line.set_data(traj2_x, traj2_y)

    frame_counter += 1

    return (
        line_l1,
        line_l2,
        joint1,
        joint2,
        mass,
        traj1_line,
        traj2_line,
        pos1_line,
        pos2_line,
        vel1_line,
        vel2_line,
        ctrl1_line,
        ctrl2_line,
        cursor_pos,
        cursor_vel,
        cursor_ctrl,
        time_text,
    )


# ----------------------------
# Keyboard controls
# ----------------------------
def on_key(event):
    global paused, frame_counter

    if event.key == " ":
        paused = not paused

    elif event.key.lower() == "r":
        paused = False
        frame_counter = 0
        reset_all()

    elif event.key == "right":
        paused = True
        frame_counter = min(frame_counter + 1, num_frames // states_step - 1)

    elif event.key == "left":
        paused = True
        frame_counter = max(0, frame_counter - 1)

    update(None)
    fig.canvas.draw_idle()


fig.canvas.mpl_connect("key_press_event", on_key)

# ----------------------------
# Animation
# ----------------------------
ani = FuncAnimation(
    fig, update, init_func=init, interval=dt * 1000 * states_step, blit=True
)

plt.tight_layout()
plt.show()
