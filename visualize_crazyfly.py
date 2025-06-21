import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def quadrotor_visualize(positions, orientations, delay, traj_step, l, cam_onboard = False):
    positions = np.asarray([positions[k] for k in range(0, len(positions), traj_step)] + [positions[-1]])
    orientations = np.asarray([orientations[k] for k in range(0, len(orientations), traj_step)] + [orientations[-1]])
    if positions.shape[0] != orientations.shape[0]:
        raise ValueError("positions and orientations must have the same length N.")

    # Define quadrotor geometry
    p_body = np.array([0.0, 0.0, 0.0])
    p_motor1 = np.array([0.0, l, 0.0])
    p_motor2 = np.array([l, 0.0, 0.0])
    p_motor3 = np.array([0.0, -l, 0.0])
    p_motor4 = np.array([-l, 0.0, 0.0])
    body_points = np.vstack([p_body, p_motor1, p_motor2, p_motor3, p_motor4])
    h = l / 4.0
    vertical_offsets = np.array([[0.0, 0.0, h]] * 4)

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
    N = positions.shape[0]
    for k in range(N):
        x = positions[k]
        R = orientations[k]
        transformed_points = (R @ body_points.T).T + x[np.newaxis, :]
        pB, pM1, pM2, pM3, pM4 = transformed_points
        motor_coords = np.vstack([pM1, pM2, pM3, pM4])

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

        ax_world.scatter(*pB, c = 'k', s = 50, label = 'Body')
        ax_world.scatter(motor_coords[:, 0], motor_coords[:, 1], motor_coords[:, 2],
                         c = 'g', s = 50, label = 'Motors')

        for i, pM in enumerate([pM1, pM2, pM3, pM4]):
            ax_world.plot([pB[0], pM[0]], [pB[1], pM[1]], [pB[2], pM[2]], 'r-', linewidth = 2)
            top_pt = pM + vertical_offsets[i]
            ax_world.plot([pM[0], top_pt[0]], [pM[1], top_pt[1]], [pM[2], top_pt[2]], 'b-', linewidth = 2)
            text_pos = pM + np.array([0, 0, h * 1.2])
            ax_world.text(*text_pos, str(i + 1), fontsize = 12, color = 'blue', ha = 'center')

        if cam_onboard:
            # Clear and update body-fixed (first-person) plot
            ax_body.cla()
            ax_body.set_title("First-person orientation view")
            ax_body.set_xlabel('X')
            ax_body.set_ylabel('Y')
            ax_body.set_zlabel('Z')
            scale = 1.5 * l
            xcor, ycor, zcor = x
            ax_body.set_xlim(xcor - scale, xcor + scale)
            ax_body.set_ylim(ycor - scale, ycor + scale)
            ax_body.set_zlim(zcor - scale, zcor + scale)

            quad_points = (R @ body_points.T).T + x[np.newaxis, :]
            pB_q, pM1_q, pM2_q, pM3_q, pM4_q = quad_points
            motor_coords_q = np.vstack([pM1_q, pM2_q, pM3_q, pM4_q])
            ax_body.scatter(*pB_q, c = 'k', s = 50, label = 'Body')
            ax_body.scatter(motor_coords_q[:, 0], motor_coords_q[:, 1], motor_coords_q[:, 2],
                            c = 'g', s = 50, label = 'Motors')

            for i, pM in enumerate([pM1_q, pM2_q, pM3_q, pM4_q]):
                ax_body.plot([pB_q[0], pM[0]], [pB_q[1], pM[1]], [pB_q[2], pM[2]], 'r-', linewidth = 2)
                top_pt = pM + vertical_offsets[i]
                ax_body.plot([pM[0], top_pt[0]], [pM[1], top_pt[1]], [pM[2], top_pt[2]], 'b-', linewidth = 2)
                text_pos = pM + np.array([0, 0, h * 1.2])
                ax_body.text(*text_pos, str(i + 1), fontsize = 12, color = 'blue', ha = 'center')

        plt.pause(delay)

    plt.ioff()
    plt.show()
