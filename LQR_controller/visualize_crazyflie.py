"""
Quadrotor (Crazyflie) visualization using FuncAnimation.

This module exposes a single public function:

    quadrotor_visualize(folder="animation")

which is called by speeds_LQR_crazyflie.py exactly as before.
All data loading and figure creation happen inside that function, so
importing this module is side-effect-free.

The CSV / JSON files are expected at:
    <this script's directory>/<folder>/crazyflie_states.csv
    <this script's directory>/<folder>/crazyflie_controls.csv
    <this script's directory>/<folder>/crazyflie_positions.csv
    <this script's directory>/<folder>/crazyflie_orientations.csv
    <this script's directory>/<folder>/crazyflie_params.json

Keyboard controls
-----------------
Space        pause / resume
R            restart from the beginning
Left / Right step one frame backward / forward (auto-pauses)
Q            quit
"""

from pathlib import Path
import json
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib.animation import FuncAnimation

# ---------------------------------------------------------------------------
# Public entry point
# ---------------------------------------------------------------------------


def quadrotor_visualize(folder: str = "animation", states_step: int = 1) -> None:
    """Load saved simulation data and show the interactive animation."""

    # -----------------------------------------------------------------------
    # Paths
    # -----------------------------------------------------------------------
    base_dir = Path(__file__).resolve().parent
    anim_dir = base_dir / folder

    # -----------------------------------------------------------------------
    # Load data
    # -----------------------------------------------------------------------
    states = np.loadtxt(anim_dir / "crazyflie_states.csv", delimiter=",")
    controls = np.loadtxt(anim_dir / "crazyflie_controls.csv", delimiter=",")
    positions = np.loadtxt(anim_dir / "crazyflie_positions.csv", delimiter=",")

    orientations_flat = np.loadtxt(
        anim_dir / "crazyflie_orientations.csv", delimiter=","
    )
    orientations = orientations_flat.reshape(-1, 3, 3)
    yaw = np.unwrap(np.arctan2(orientations[:, 1, 0], orientations[:, 0, 0]))

    with open(anim_dir / "crazyflie_params.json", "r") as f:
        params = json.load(f)

    dt = params["dt"]
    l = params["arm_length"]
    body_yaw0 = params["body_yaw0"]
    u_max = np.asarray(params["u_max"])

    # -----------------------------------------------------------------------
    # Derived quantities
    # -----------------------------------------------------------------------
    num_frames = positions.shape[0]
    display_frames = (num_frames - 1) // states_step + 1
    total_time = (num_frames - 1) * dt

    # Body-frame geometry (before world rotation)
    cy, sy = np.cos(body_yaw0), np.sin(body_yaw0)
    body_orient0 = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])

    p_body = np.array([0.0, 0.0, 0.0])
    p_antenna = l / 4 * np.array([-1.0 / np.sqrt(2), 1.0 / np.sqrt(2), 0.0])
    p_motor1 = np.array([0.0, l, 0.0])
    p_motor2 = np.array([l, 0.0, 0.0])
    p_motor3 = np.array([0.0, -l, 0.0])
    p_motor4 = np.array([-l, 0.0, 0.0])

    body_points = (
        body_orient0
        @ np.vstack([p_body, p_antenna, p_motor1, p_motor2, p_motor3, p_motor4]).T
    )  # (3, 6)

    # 3-D axis bounds
    x_min, y_min, z_min = positions.min(axis=0) - l
    z_min = max(z_min, 0.0)
    x_max, y_max, z_max = positions.max(axis=0) + l
    ax_range = max(x_max - x_min, y_max - y_min)
    ax_margin = 0.1

    def _ylim(arr, margin=0.1):
        mn, mx = float(arr.min()), float(arr.max())
        span = (mx - mn) or 1.0
        return mn - margin * span, mx + margin * span

    # -----------------------------------------------------------------------
    # Figure layout
    # -----------------------------------------------------------------------
    fig = plt.figure(figsize=(16, 9))
    fig.suptitle("Crazyflie LQR simulation", fontsize=13)

    ax3d = fig.add_subplot(1, 2, 1, projection="3d")
    ax_x = fig.add_subplot(2, 4, 3)
    ax_v = fig.add_subplot(2, 4, 4)
    ax_om = fig.add_subplot(2, 4, 7)
    ax_u = fig.add_subplot(2, 4, 8)

    # 3-D view
    ax3d.set_title("3-D trajectory")
    ax3d.set_xlabel("X (m)")
    ax3d.set_ylabel("Y (m)")
    ax3d.set_zlabel("Z (m)")
    ax3d.view_init(elev=45, azim=30)
    ax3d.set_xlim(x_min - ax_margin, x_min + ax_range + ax_margin)
    ax3d.set_ylim(y_min - ax_margin, y_min + ax_range + ax_margin)
    ax3d.set_zlim(0.0, z_max + ax_margin)

    # Time-history axes
    colors_xyz = ("r", "g", "b")
    color_yaw = "k"
    colors_u = ("r", "g", "b", "m")

    ax_x.set_title("Position  $r_w$  (m)")
    ax_x.set_xlabel("t (s)")
    ax_x.set_ylabel("m")
    ax_x.set_xlim(0.0, total_time)
    ax_x.set_ylim(*_ylim(positions))

    ax_x_r = ax_x.twinx()
    ax_x_r.set_ylabel("yaw (rad)")
    ax_x_r.set_ylim(*_ylim(yaw))

    ax_v.set_title("Body velocity  $v_b$  (m/s)")
    ax_v.set_xlabel("t (s)")
    ax_v.set_ylabel("m/s")
    ax_v.set_xlim(0.0, total_time)
    ax_v.set_ylim(*_ylim(states[:, 7:10]))

    ax_om.set_title("Angular velocity  $ω_b$  (rad/s)")
    ax_om.set_xlabel("t (s)")
    ax_om.set_ylabel("rad/s")
    ax_om.set_xlim(0.0, total_time)
    ax_om.set_ylim(*_ylim(states[:, 10:13]))

    ax_u.set_title("Motor speeds  u  (rad/s)")
    ax_u.set_xlabel("t (s)")
    ax_u.set_ylabel("rad/s")
    ax_u.set_xlim(0.0, total_time)
    ax_u.set_ylim(*_ylim(controls))

    # Legends
    for ax, labels, cols in [
        (ax_x, ("x", "y", "z"), colors_xyz),
        (ax_v, ("vx", "vy", "vz"), colors_xyz),
        (ax_om, ("ωx", "ωy", "ωz"), colors_xyz),
        (ax_u, ("u1", "u2", "u3", "u4"), colors_u),
    ]:
        for lbl, col in zip(labels, cols):
            ax.plot([], [], color=col, lw=1, label=lbl)
        ax.legend(fontsize=7, loc="upper right")
    ax_x_r.plot([], [], color=color_yaw, lw=1, label="yaw")
    ax_x_r.legend(fontsize=7, loc="lower right")

    # -----------------------------------------------------------------------
    # Artists – 3-D body
    # -----------------------------------------------------------------------
    trace_sc = ax3d.scatter([], [], [], c="k", s=10, zorder=3)
    body_sc = ax3d.scatter([], [], [], c="k", s=60, zorder=5)
    antenna_sc = ax3d.scatter([], [], [], c="k", s=20, zorder=5)
    motor_sc = ax3d.scatter([], [], [], c="g", s=60, zorder=5)

    arm_lines = [ax3d.plot([], [], [], "r-", lw=2)[0] for _ in range(4)]
    ctrl_lines = [ax3d.plot([], [], [], "b-", lw=2)[0] for _ in range(4)]
    ant_line = ax3d.plot([], [], [], "k-", lw=1)[0]
    ctrl_texts = [
        ax3d.text(0, 0, 0, str(i + 1), fontsize=9, color="blue", ha="center")
        for i in range(4)
    ]
    time_text = ax3d.text2D(0.04, 0.93, "", transform=ax3d.transAxes, fontsize=9)

    # -----------------------------------------------------------------------
    # Artists – time-history lines
    # -----------------------------------------------------------------------
    pos_lines = [ax_x.plot([], [], color=c, lw=1)[0] for c in colors_xyz]
    yaw_line = ax_x_r.plot([], [], color="k", lw=1.5, label="yaw")[0]
    vel_lines = [ax_v.plot([], [], color=c, lw=1)[0] for c in colors_xyz]
    om_lines = [ax_om.plot([], [], color=c, lw=1)[0] for c in colors_xyz]
    u_lines = [ax_u.plot([], [], color=c, lw=1)[0] for c in colors_u]

    cursors = [
        ax_x.axvline(0, ls="--", lw=0.8, color="k"),
        ax_v.axvline(0, ls="--", lw=0.8, color="k"),
        ax_om.axvline(0, ls="--", lw=0.8, color="k"),
        ax_u.axvline(0, ls="--", lw=0.8, color="k"),
    ]

    # -----------------------------------------------------------------------
    # Mutable state (use a dict so closures can mutate it)
    # -----------------------------------------------------------------------
    trace_buf = []
    trace_max = max(1, num_frames // 4)
    anim_state = {"paused": False, "frame": 0}

    # -----------------------------------------------------------------------
    # Update
    # -----------------------------------------------------------------------
    def update(_):
        k = anim_state["frame"] * states_step

        if k >= num_frames:
            anim_state["frame"] = 0
            k = 0
            trace_buf.clear()

        r = positions[k]
        R = orientations[k]
        u = controls[k]

        # Body points in world frame
        pts_world = (R @ body_points).T + r  # (6, 3)
        pB, pA, pM1, pM2, pM3, pM4 = pts_world
        pMs = np.vstack([pM1, pM2, pM3, pM4])  # (4, 3)

        # Motor-speed bars
        z_off = (l * u / u_max).clip(0.0, l)
        ctrl_pts = np.copy(body_points[:, 2:])  # motor columns
        ctrl_pts[2, :] += z_off
        pCs = (R @ ctrl_pts).T + r  # (4, 3)

        # Trace
        if len(trace_buf) >= trace_max:
            trace_buf.pop(0)
        trace_buf.append(r.copy())
        tr = np.array(trace_buf)

        # Update 3-D artists
        trace_sc._offsets3d = (tr[:, 0], tr[:, 1], tr[:, 2])
        body_sc._offsets3d = ([pB[0]], [pB[1]], [pB[2]])
        antenna_sc._offsets3d = ([pA[0]], [pA[1]], [pA[2]])
        motor_sc._offsets3d = (pMs[:, 0], pMs[:, 1], pMs[:, 2])

        ant_line.set_data([pB[0], pA[0]], [pB[1], pA[1]])
        ant_line.set_3d_properties([pB[2], pA[2]])

        for i in range(4):
            arm_lines[i].set_data([pB[0], pMs[i, 0]], [pB[1], pMs[i, 1]])
            arm_lines[i].set_3d_properties([pB[2], pMs[i, 2]])

            ctrl_lines[i].set_data([pMs[i, 0], pCs[i, 0]], [pMs[i, 1], pCs[i, 1]])
            ctrl_lines[i].set_3d_properties([pMs[i, 2], pCs[i, 2]])

            ctrl_texts[i].set_position((pCs[i, 0], pCs[i, 1]))
            ctrl_texts[i].set_3d_properties(pCs[i, 2], zdir=None)

        time_text.set_text(f"t = {k * dt:.2f} s")

        # Update time-history plots
        idx = np.arange(0, k + 1, states_step)
        t_h = idx * dt
        rw_h = positions[idx]
        yaw_h = yaw[idx]
        vb_h = states[idx, 7:10]
        om_h = states[idx, 10:13]
        u_h = controls[idx]

        for i, ln in enumerate(pos_lines):
            ln.set_data(t_h, rw_h[:, i])
        yaw_line.set_data(t_h, yaw_h)
        for i, ln in enumerate(vel_lines):
            ln.set_data(t_h, vb_h[:, i])
        for i, ln in enumerate(om_lines):
            ln.set_data(t_h, om_h[:, i])
        for i, ln in enumerate(u_lines):
            ln.set_data(t_h, u_h[:, i])

        t_now = k * dt
        for cur in cursors:
            cur.set_xdata([t_now, t_now])

        if not anim_state["paused"]:
            anim_state["frame"] += 1

        return (
            trace_sc,
            body_sc,
            antenna_sc,
            motor_sc,
            ant_line,
            time_text,
            *arm_lines,
            *ctrl_lines,
            *ctrl_texts,
            *pos_lines,
            yaw_line,
            *vel_lines,
            *om_lines,
            *u_lines,
            *cursors,
        )

    # -----------------------------------------------------------------------
    # Keyboard handler
    # -----------------------------------------------------------------------
    def on_key(event):
        key = (event.key or "").lower()

        if key == " ":
            anim_state["paused"] = not anim_state["paused"]

        elif key == "r":
            anim_state["paused"] = False
            anim_state["frame"] = 0
            trace_buf.clear()

        elif key == "up":
            anim_state["paused"] = True
            anim_state["frame"] = min(anim_state["frame"] + 1, display_frames - 1)
            update(None)
            fig.canvas.draw_idle()

        elif key == "down":
            anim_state["paused"] = True
            anim_state["frame"] = max(anim_state["frame"] - 1, 0)
            update(None)
            fig.canvas.draw_idle()

        elif key == "q":
            plt.close("all")

    fig.canvas.mpl_connect("key_press_event", on_key)

    # -----------------------------------------------------------------------
    # Run
    # -----------------------------------------------------------------------
    plt.tight_layout()

    ani = FuncAnimation(  # noqa: F841  keep reference alive
        fig,
        update,
        interval=dt * 1000,
        blit=False,
        cache_frame_data=False,
    )

    plt.show()


# ---------------------------------------------------------------------------
# Allow running the script directly as well
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    quadrotor_visualize()
