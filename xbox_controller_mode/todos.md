TODOs

- Along with the Position and Orientation of the drone:
    - Show Battery Level
    - Show Motors Control Inputs as a percentage (0 - 65535 PWM -> 0 - 100 % percentage)

- The lighthouse station numbers should be shown in the scene, above their origin, using the scene font, in the form "LH #num". Also, the drone should have the text "drone" above its center, and the world frame the text world frame above its origin.

- The motor control inputs should be visualized on the quadrotor.

- Add KEYDOWN events for:
    - show/hide world frame -> shift + w
    - show/hide quadrotor body -> shift + d
    - show/hide quadrotor frame -> shift + f
    - show/hide lighthouse stations -> shift + l
    - show/hide hud_font text -> shift + h
    - show/hide scene_font text -> shift + t
    - retry to establish connection -> ctrl + r

- Info menu (using the hud font) about the various bindings defined above, that should appear in the right side of the window. In the left side of the window should be written "Press [i] for more info", and pressing i should open the info menu.

- Move crazyflie stuff in a seperate python file, in order to keep the visualization and drone stuff apart.

- Show in the hud, above Camera look-at, the resolution of the ground, like: "Ground Grid : {GRID_CELL_M} m per cell" or something like that.

- Show in the hud not only the connection state, position, orientation, battery level and motor controls for the drones, but also the estimator and the controller used. They are the stabilizer.controller and stabilizer.estimator parameters. Search for the numbers and the correspondance to the controllers and estimators names in order to include both the number and name for each.

- Better groupings for the logging variables and parameters in crazyflie.py. Right now I have only one, called DroneStateControl.

- Integration of multiple crazyflie drones. I want them to appear in the scene with labels "CF 1", "CF 2", "Cf 3", etc. The default poses should be in positions (-1, 1, 0), (1, 1, 0), (1, -1, 0), (-1, -1, 0), (-2, 2, 0), (2, 2, 0), and so on based on the number of crazyflie addresses provided. The positions, orientations, battery levels, and every info I specified that must be shown for a drone in the hud, should apppear for all provided drones. Also, it should be written in the beginning of the info for every crazyflie drone: "CF {num} : {full_address}" according to the addresses list provided. You may decrease the hud_font if necessary, and automatically appear a scrollbar if the text gets too large and exceeds the height of the pygame window. In the hud, separate the info for each drone with a thin black line separator.

- Possibility to add a crazyflie with ctrl + a. A menu should appear in the center of the screen containing fields for the Radio ID (default 0), Channel (default 80), Datarate (default 2M), and Address (default E7E7E7E7E7). The new crazyflie should be numbered according to the current number of the existing drones, and should initially spawn in the scene in its corresponding (based on its number) default pose. After the addition, the crazyflie should wait for the user to establish a connection. Also, possibility to delete a crazyflie with ctrl + d. A menu should appear with a field asking for the number of the crazyflie that should be deleted. After the deletion, the specified drones should disconnect and disappear from the scene, and the other drones should keep their numbers, even though gaps could appear in the numerical order.

- Add shift + numkey binding to show/hide specific drones in the scene. The binding shift + 0 should be used to show/hide all drones. Also add ctrl + numkey binding to indicate the real drone by blinking the builtin led 3 times in the span of 3 seconds (1 blink per second), using the parameter "led.bitmask".

- The ctrl + r binding should be used to establish a connection with a drone. A menu should appear, asking for the number of the drone to be connected/reconnected, with the 0 choice used for the connection/reconnection of all the current drones. In this sense, the CLI parsing --connect does not have a reason to exist, when the app opens, the default should be for all the drones to be disconnected. If a drone disconnects for any reason, it should immediately be indicated in the hud, in the connection state info.

- Add ctrl + c binding . I have in mind something like the high level control appearing in the standard cfclient application. You can use whatever needed from MotionCommander, PositionCommander and HighLevelCommander.
