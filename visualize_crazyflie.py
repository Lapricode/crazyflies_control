import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def quadrotor_visualize(positions, orientations, control_inputs, max_control_input, delay, traj_step, l, body_yaw0, cam_onboard = False):
    positions = np.asarray([positions[k] for k in range(0, len(positions), traj_step)] + [positions[-1]])
    N = positions.shape[0]
    trace_positions = []
    orientations = np.asarray([orientations[k] for k in range(0, len(orientations), traj_step)] + [orientations[-1]])
    if positions.shape[0] != orientations.shape[0]:
        raise ValueError("positions and orientations must have the same length N.")

    # Define quadrotor geometry
    body_orient0 = np.array([[np.cos(body_yaw0), -np.sin(body_yaw0), 0.0], \
                            [np.sin(body_yaw0), np.cos(body_yaw0), 0.0], \
                            [0.0, 0.0, 1.0]])
    p_body = np.array([0.0, 0.0, 0.0])
    p_antenna = l/4 * np.array([-1.0 / np.sqrt(2), 1.0 / np.sqrt(2), 0.0])
    p_motor1 = np.array([0.0, l, 0.0])
    p_motor2 = np.array([l, 0.0, 0.0])
    p_motor3 = np.array([0.0, -l, 0.0])
    p_motor4 = np.array([-l, 0.0, 0.0])
    body_points = body_orient0 @ np.vstack([p_body, p_antenna, p_motor1, p_motor2, p_motor3, p_motor4]).T
    controls_max_length = l

    # Setup plots
    fig = plt.figure(figsize = (14, 8))
    if cam_onboard:
        ax_world = fig.add_subplot(121, projection = '3d')
    else:
        ax_world = fig.add_subplot(111, projection = '3d')
    ax_world.set_title("Crazyflie trajectory")
    ax_world.view_init(elev = 45, azim = 45)
    all_xyz = positions.reshape(-1, 3)
    x_min, y_min, z_min = all_xyz.min(axis = 0) - l
    z_min = max(z_min, 0)
    x_max, y_max, z_max = all_xyz.max(axis = 0) + l
    ax_world_range = max(x_max - x_min, y_max - y_min)
    ax_world_margin = 0.3

    if cam_onboard:
        ax_body = fig.add_subplot(122, projection = '3d')
        ax_body.set_title("First-person orientation view")
        ax_body.view_init(elev = 0, azim = 0)

    # Animation
    plt.ion()
    fig.canvas.mpl_connect('key_release_event', lambda event: [exit(0) if event.key == "q" else None])
    for k in range(N):
        r = positions[k]
        R = orientations[k]
        u = control_inputs[k]
        transformed_body_points = (R @ body_points).T + r[np.newaxis, :]
        pB, pA, pM1, pM2, pM3, pM4 = transformed_body_points
        show_controls_lengths = controls_max_length * (u / max_control_input)**2
        show_controls_points = np.copy(body_points[:, 2:])
        show_controls_points[2, :] = show_controls_points[2, :] + show_controls_lengths
        pC1, pC2, pC3, pC4 = (R @ show_controls_points).T + r[np.newaxis, :]
        pMs = np.vstack([pM1, pM2, pM3, pM4])
        pCs = np.vstack([pC1, pC2, pC3, pC4])
        if k >= N//4:
            trace_positions.pop(0)
        trace_positions.append(r)
        trace_positions_array = np.array(trace_positions)

        # Clear and update world plot
        ax_world.cla()
        ax_world.set_title("Crazyflie trajectory")
        ax_world.set_xlabel('X')
        ax_world.set_ylabel('Y')
        ax_world.set_zlabel('Z')
        # ax_world.set_xlim(x_min, x_max)
        # ax_world.set_ylim(y_min, y_max)
        # ax_world.set_zlim(z_min, z_max)
        ax_world.set_xlim(x_min - ax_world_margin, x_min + ax_world_range + ax_world_margin)
        ax_world.set_ylim(y_min - ax_world_margin, y_min + ax_world_range + ax_world_margin)
        ax_world.set_zlim(0., z_max + ax_world_margin)
        # ax_world.set_aspect("equal")

        ax_world.scatter(trace_positions_array[:, 0], trace_positions_array[:, 1], trace_positions_array[:, 2], c = 'k', s = 20, label = 'trace')
        ax_world.scatter(*pB, c = 'k', s = 50, label = 'body')
        ax_world.scatter(*pA, c = 'k', s = 20, label = 'body')
        ax_world.scatter(pMs[:, 0], pMs[:, 1], pMs[:, 2], c = 'g', s = 50, label = 'motors')

        for i in range(4):
            ax_world.plot([pB[0], pMs[i][0]], [pB[1], pMs[i][1]], [pB[2], pMs[i][2]], 'r-', linewidth = 2)
            ax_world.plot([pB[0], pA[0]], [pB[1], pA[1]], [pB[2], pA[2]], 'k-', linewidth = 1)
            ax_world.plot([pMs[i][0], pCs[i][0]], [pMs[i][1], pCs[i][1]], [pMs[i][2], pCs[i][2]], 'b-', linewidth = 2)
            text_pos = pCs[i]
            ax_world.text(*text_pos, str(i + 1), fontsize = 12, color = 'blue', ha = 'center')

        if cam_onboard:
            # Clear and update body-fixed (first-person) plot
            ax_body.cla()
            ax_body.set_title("First-person orientation view")
            ax_body.set_xlabel('X')
            ax_body.set_ylabel('Y')
            ax_body.set_zlabel('Z')
            view_scale = 1.5 * l
            xcor, ycor, zcor = r
            ax_body.set_xlim(xcor - view_scale, xcor + view_scale)
            ax_body.set_ylim(ycor - view_scale, ycor + view_scale)
            ax_body.set_zlim(zcor - view_scale, zcor + view_scale)

            quad_body_points = (R @ body_points).T + r[np.newaxis, :]
            pB_q, pA_q, pM1_q, pM2_q, pM3_q, pM4_q = quad_body_points
            pMs_q = np.vstack([pM1_q, pM2_q, pM3_q, pM4_q])
            ax_body.scatter(*pB_q, c = 'k', s = 50, label = 'body')
            ax_body.scatter(*pA_q, c = 'k', s = 20, label = 'body')
            ax_body.scatter(pMs[:, 0], pMs[:, 1], pMs[:, 2], c = 'g', s = 50, label = 'motors')

            for i in range(4):
                ax_body.plot([pB[0], pMs[i][0]], [pB[1], pMs[i][1]], [pB[2], pMs[i][2]], 'r-', linewidth = 2)
                ax_body.plot([pB[0], pA[0]], [pB[1], pA[1]], [pB[2], pA[2]], 'k-', linewidth = 1)
                ax_body.plot([pMs[i][0], pCs[i][0]], [pMs[i][1], pCs[i][1]], [pMs[i][2], pCs[i][2]], 'b-', linewidth = 2)
                text_pos = pCs[i]
                ax_body.text(*text_pos, str(i + 1), fontsize = 12, color = 'blue', ha = 'center')

        plt.pause(delay)

    plt.ioff()
    plt.show()
