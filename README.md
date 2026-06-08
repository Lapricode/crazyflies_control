1. I have a few slides, `crazyflies_getting_started`, with general information and initial instructions about the Crazyflie and its setup. The dynamic model of the Crazyflie is also described in detail.

2. For the localization of the Crazyflie, we use the Lighthouse Positioning System. More about it, its installation, and calibration can be found at the following links:
    - https://www.bitcraze.io/documentation/system/positioning/ligthouse-positioning-system/
    - https://www.bitcraze.io/documentation/tutorials/getting-started-with-lighthouse/

    Calibration is done through `cfclient` by opening the `Lighthouse Positioning` tab. It is a good idea to save the arena configuration by clicking `Save system config` after the `geometry estimation` process is completed.

3. I have created plots in GeoGebra (file `crazyflie_dynamics_curves.ggb`) for the functional relationships between various useful quantities of the Crazyflie dynamic model. For example (all of the following relationships refer to only 1 of the 4 motors):
    - rotor speed (`rad/sec`) vs. PWM:

        $$
        \omega_{r} = \sqrt{8 \cdot 10^{-4} \cdot \mathrm{PWM}^2 + 53.33 \cdot \mathrm{PWM}}
        $$

    - PWM vs. rotor speed (`rad/sec`):

        $$
        \mathrm{PWM} = -33333.0 + \sqrt{1250.0 \cdot \omega_{r}^2 + 1111111111.0}
        $$

    - thrust (`N`) vs. rotor speed (`rad/sec`):

        $$
        F_{i} = k_{f} \omega_{r}^2 = 2.25 \cdot 10^{-8} \omega_{r}^2
        $$

    - thrust (`N`) vs. PWM:

        $$
        F_{i} = 1.8 \cdot 10^{-11} \cdot \mathrm{PWM}^2 + 1.2 \cdot 10^{-6} \cdot \mathrm{PWM}
        $$

    - PWM vs. normalized thrust (`N`):

        $$
        \mathrm{PWM} = -33333.0 + \sqrt{8663836225.0 \cdot \mathrm{norm_{F_{i}}} + 1111111111.0}
        $$

    - normalized PWM vs. normalized thrust (`N`):
        $$
        \mathrm{norm_{PWM}} = -0.50863 + \sqrt{0.25871 + 2.01727 \cdot \mathrm{norm_{F_{i}}}}
        $$

    I consulted the following works:
    - https://lup.lub.lu.se/luur/download?func=downloadFile&recordOId=8905295&fileOId=8905299 (specifically Appendix A)
    - https://www.research-collection.ethz.ch/handle/20.500.11850/214143 (specifically Chapter 3)

4. The original firmware can be found at https://github.com/bitcraze/crazyflie-firmware. To create your own controller, there are detailed instructions at: https://www.bitcraze.io/2023/02/adding-an-estimator-or-controller/.

    Here I also record some of my observations (they are also in my fork: https://github.com/Lapricode/crazyflie-firmware):
    - In the folder `examples/my_controller_estimator/`:
        - create `src/my_controller.c` (here I have the necessary `appMain`; the functions `ControllerOutOfTreeInit`, `ControllerOutOfTreeTest`, and `ControllerOutOfTree` are required)
        - make the appropriate changes in `app-config` and `src/Kbuild`

    - In the folder `src/modules/src`:
        - the `controlMotors` function in `stabilizer.c` calls `powerDistribution` from the file `power_distribution_quadrotor.c`, which, depending on the selected `controlMode`, calls the appropriate function that sends PWM signals to the motors (I have implemented `powerDistributionForce` based on `powerDistributionForceTorque`).

    - In the file `src/modules/src/controller/controller.c`:
        - I disabled the forced controller by commenting out the lines `#elif defined(CONFIG_CONTROLLER_OOT) \ #define CONTROLLER ControllerTypeOot`, in order to make it possible to change the controller through `cfclient` or the Python API.

5. For the Python API, you can find useful material in my repository: https://github.com/Lapricode/crazyflies_control. In particular:
    - `speeds_LQR_crazyflie.py` and `thrusts_LQR_crazyflie.py` for my LQR controller with rotor speeds and thrusts, respectively
    - `visualize_crazyflie.py` for simulation with `matplotlib`

    In the folder `crazyflies_control/crazyflies_python_api`:
    - `position_control.py` contains the high-level commander for setting position and yaw angle. Obviously, with a few modifications, it can also be used for trajectory tracking. It also shows how to monitor specific log variables and set parameters, such as `stabilizer.controller` (this is the index of the `controllerFunctions` array in the file `src/modules/src/controller/controller.c` of the firmware; the parameters are also defined through `cfclient`, from the `Parameters` tab)
    - `swarm_choreography.py` contains a simple script for executing a circular trajectory with a swarm of 4 Crazyflies
    - `check_uris.py` is for detecting the URIs of the Crazyflies; this can also be done by connecting the Crazyflie via USB through `cfclient` and going to `Connect -> Configure 2.x`
