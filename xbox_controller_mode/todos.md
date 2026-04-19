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

- First of all, I need you to fix a small bug. If I put for example 2 drones in the scene and I delete the number 1, then the next that I add, that is the number 3, is placed on the default position of the exisiting number 2, and does not go in the default position of number 3. Also, I want the default positions to continue for more than 8 drones, so do not have a list for the default positions, figure them out from the number of drone that is added.

- Add ctrl + c binding to create a simple high - level controller for the drones in the scene. I have in mind something like the high-level control menu appearing in the standard cfclient application, but a bit modified. You can use whatever needed from MotionCommander, PositionCommander and HighLevelCommander. Specifically, I want the control menu, that should appear in the bottom right corner of the pygame window, to include the following: a) A field for choosing the current crazyflie controlled using its number (if 0, then all the crazyflies are controlled simultaneously). b) Fields to write the translation step size (in m) used, let's call it dx, and rotational step size (in deg) used, let's call it dq. c) A lift off button, for the drone to rise up to dx height (bind it to f key also). c) A land button (bind it to l key also). d) Increase / Decrease yaw (by dq) buttons (bind them to a and d keys also). e) Increase / Decrease height (by dx) buttons (bind them to w and s keys also). f) Go forward/backward (by dx) buttons (bind them to up/down arrow keys also). g) Go sideways to the left/right (by dx) buttons (bind them to left/right arrow keys also). I would prefer to not use Swarm, instead use SyncCrazyflie.

- Now, this may need a different python file. I want to be able to control my crazyflies using an xbox controller. First of all, if an xbox controller is connected, then I would like to be indicated in the hud, below the "[i] key bindings". I want to define an input mapping. Initially, the xbox controller I have has in the left side a (left) joystick, the four arrow buttons (up, down, left, right), and the buttons L1, L2, in the right side a (right) joystick, the buttons X, Y, A, B, and the buttons R1, R2, and in the center the buttons select, start, and mode. So, the mapping between these controls and the various motion and other commands should be: 0) alternate between manual / auto flight mode -> mode button (manual is basically for controlling thrust manually, and auto is for maintaining the same flight height and world pose without needing human input). 1) increase thrust -> left joystick go up (max thrust is 65535), manual + auto mode. 2) decrease thrust -> left joystick go down (min thrust is 0), manual + auto mode. 3) increse yaw -> left joystick go left (max yaw is 360 deg/sec), manual + auto mode. 4) decrease yaw -> left joystick go right (min yaw is -360 deg/sec), manual + auto mode. 5) increase roll -> right joystick go right (max roll is 25 deg), manual + auto mode. 6) decrease roll -> right joystick go left (min roll is -25 deg), manual + auto mode. 7) increase pitch -> right joystick go up (max pitch is 25 deg), manual + auto mode. 8) decrease pitch -> right joystick go down (min pitch is -25 deg), manual + auto mode. 9) takeoff to 0.5 m height -> start button, auto mode only. 10) land -> start button again, auto mode only. 11) emergency stop -> L2 button, manual + auto mode. 12) go up increasing height by 0.5 m -> up arrow button, auto mode only. 13) go down decreasing height by 0.5 m -> down arrow button, auto mode only. 14) go sideways to the right by 0.5 m -> right arrow button, auto mode only. 15) go sideways to the left by 0.5 m -> left arrow button, auto mode only. 16) change crazyflie that is controlled by decreasing/increasing numbers with L1/R1 buttons and finally choosing with select button (if an other input is given before pressing select, then change operation gets cancelled and the number goes back to the current controlled drone), manual + auto mode (the previous controlled drone should go automatically into auto mode). 17) blink builtin led on the current controlled real drone -> R2 button, manual + auto mode. 18) for all the previous numbers for roll, pitch, go up, go down (25 deg, 0.5 m for example), there should be small/big movement versions -> A/B buttons, manual + auto mode.

- Limit the number of crazyflies that can exist in the scene simultaneously to 8. So, if the user tries to add another drone with ctrl + a, the modal window should not open, and the terminal should warn accordingly.

- In the hud, in the section for the lighthouse stations, there should be written the world positions of the existing lighthouses.

- Add ctrl + s binding for saving the current drones configuration for all the existing drones in the scene, in a yaml file called crazyflies_config.yaml, specifically their full addresses. This yaml file may contain many drones configurations, numbered in numerical order 1-indexed. Add also ctrl + l binding for loading a crazyflie config, where the user should be asked about the desirable crazyflie config number to load in the scene.

- The configs for the lighthouse stations and the crazyflies should be moved in a folder called config, and the code should be modified to reflect this organization change.

- Add ctrl + p binding for changing parameters for specific drones. There should be a field for the number of the desirable drone, and for starters you should have fields for these parameters (I may expand this later).
    - Led Value: 0-255
    - Controller: the user types the controller number
    - Estimator: the user types the estimator number

- Add alt + numkey binding (numkey 1-8) for tracking a specific drone (it should go together with the shift bindings, in the info panel). This means that the camera lookat point should be following the origin of the specified crazyflie. Also, add alt + 0 binding for resetting to manual mode, where the camera lookat point is contrlled by panning with the mouse, as before.

- Add shift + c binding for showing the console/terminal prints. It should open/close a window in the bottom middle of the pygame window. The console window should not cover (in other words it should not be above) the hud or the flight controller, it should be the proper length to appear separated.
