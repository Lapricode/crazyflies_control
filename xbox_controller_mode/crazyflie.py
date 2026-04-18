#!/usr/bin/env python3
"""
Crazyflie connection, state estimation, and telemetry.
"""


import math
import time
import threading
import logging
import numpy as np

try:
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    from cflib.crazyflie.log import LogConfig
    CFLIB_AVAILABLE = True
except ImportError:
    CFLIB_AVAILABLE = False


# ==============================================================================
# CONFIGURATION
# ==============================================================================

LOG_PERIOD_MS = 10       # sampling period in msec for log variables
MAX_PWM       = 65535    # maximum motor PWM value (16-bit)
VBAT_MIN      = 3.0      # battery voltage at 0 % (V)
VBAT_MAX      = 4.2      # battery voltage at 100 % (V)

# stabilizer.estimator parameter value → name mapping
# source: Crazyflie firmware stabilizer_types.h
ESTIMATOR_NAMES = {
    0: "Any",
    1: "Complementary",
    2: "Kalman (EKF)",
    3: "UKF",
}

# stabilizer.controller parameter value → name mapping
# source: Crazyflie firmware controller_autoselect.h
CONTROLLER_NAMES = {
    0: "Any",
    1: "PID",
    2: "Mellinger",
    3: "INDI",
    4: "Brescianini",
}


# ==============================================================================
# HELPERS
# ==============================================================================

def _euler_zyx_to_R(roll_deg, pitch_deg, yaw_deg):
    """
    ZYX Euler → body rotation matrix wrt world
    """
    r, p, y = map(math.radians, [roll_deg, pitch_deg, yaw_deg])
    Rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    Ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    Rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    return Rz @ Ry @ Rx


# ==============================================================================
# THREAD-SAFE DRONE STATE
# ==============================================================================

class DroneState:
    """
    Crazyflie state:
        - pwb          : body position wrt world (m)
        - Rwb          : body rotation wrt world, columns = body axes in world
        - roll         : body roll angle (deg)
        - pitch        : body pitch angle (deg)
        - yaw          : body yaw angle (deg)
        - vbat         : battery voltage (V)
        - motors_pwm   : motor PWM values [m1, m2, m3, m4] (0 - 65535)
        - estimator_id : stabilizer.estimator parameter value (read once at connection)
        - controller_id: stabilizer.controller parameter value (read once at connection)
        - connected    : boolean indication for the connection state
    """

    def __init__(self, default_pos = None):
        self._lock         = threading.Lock()
        self.pwb           = np.asarray(default_pos, float) if default_pos is not None else np.zeros(3)
        self.Rwb           = np.eye(3)
        self.roll          = 0.0
        self.pitch         = 0.0
        self.yaw           = 0.0
        self.vbat          = 0.0
        self.motors_pwm    = np.zeros(4)
        self.estimator_id  = 0
        self.controller_id = 0
        self.connected     = False

    def update(self, pwb, Rwb, roll, pitch, yaw, vbat, motors_pwm):
        with self._lock:
            self.pwb        = np.asarray(pwb, float)
            self.Rwb        = np.asarray(Rwb, float)
            self.roll       = float(roll)
            self.pitch      = float(pitch)
            self.yaw        = float(yaw)
            self.vbat       = float(vbat)
            self.motors_pwm = np.asarray(motors_pwm, float)

    def set_control_config(self, estimator_id, controller_id):
        """
        Set estimator and controller IDs (read once from parameters at connection time).
        """
        with self._lock:
            self.estimator_id  = int(estimator_id)
            self.controller_id = int(controller_id)

    def get(self):
        with self._lock:
            return (self.pwb.copy(), self.Rwb.copy(),
                    self.roll, self.pitch, self.yaw,
                    self.vbat, self.motors_pwm.copy())

    def get_control_config(self):
        with self._lock:
            return self.estimator_id, self.controller_id

    def battery_pct(self):
        with self._lock:
            return max(0.0, min(100.0, (self.vbat - VBAT_MIN) / (VBAT_MAX - VBAT_MIN) * 100.0))

    def estimator_name(self):
        with self._lock:
            return ESTIMATOR_NAMES.get(self.estimator_id, f"Unknown ({self.estimator_id})")

    def controller_name(self):
        with self._lock:
            return CONTROLLER_NAMES.get(self.controller_id, f"Unknown ({self.controller_id})")


# ==============================================================================
# CRAZYFLIE CONNECTION THREAD
# ==============================================================================

class CrazyflieThread(threading.Thread):
    """
    Background thread that connects to a Crazyflie and streams
    state estimates into a shared DroneState object.

    Log groups:
        "StateEst"  : stateEstimate.{x, y, z, roll, pitch, yaw}   (6 floats, 24 B)
        "Power"     : pm.vbat                                       (1 float,   4 B)
        "Motors"    : motor.{m1, m2, m3, m4}                       (4 uint16,  8 B)

    Parameters (read once at connection):
        stabilizer.estimator
        stabilizer.controller

    Note: motor variable names and types may vary across firmware versions.
    """

    def __init__(self, uri, state: DroneState):
        super().__init__(daemon = True, name = f"cf-{uri}")
        self.uri   = uri
        self.state = state
        self._stop = threading.Event()
        self._cf   = None  # live Crazyflie handle, set inside run() and used by blink_led()

    def run(self):
        if not CFLIB_AVAILABLE:
            print("[CFLIB] cflib not installed - running without drone.")
            return
        logging.basicConfig(level = logging.ERROR)
        cflib.crtp.init_drivers()
        try:
            with SyncCrazyflie(self.uri, cf = Crazyflie(rw_cache = "./cache")) as scf:
                self._cf = scf.cf
                self.state.connected = True
                print(f"[CFLIB] Connected: {self.uri}")

                # read stabilizer parameters once at connection time
                try:
                    est_id  = int(float(scf.cf.param.get_value("stabilizer.estimator")))
                    ctrl_id = int(float(scf.cf.param.get_value("stabilizer.controller")))
                    self.state.set_control_config(est_id, ctrl_id)
                    print(f"[CFLIB] Estimator: {est_id} ({ESTIMATOR_NAMES.get(est_id, '?')}), "
                          f"Controller: {ctrl_id} ({CONTROLLER_NAMES.get(ctrl_id, '?')})")
                except Exception as exc:
                    print(f"[CFLIB] Could not read stabilizer params: {exc}")

                # log group 1 - state estimate
                lg_state = LogConfig(name = "StateEst", period_in_ms = LOG_PERIOD_MS)
                lg_state.add_variable("stateEstimate.x",     "float")
                lg_state.add_variable("stateEstimate.y",     "float")
                lg_state.add_variable("stateEstimate.z",     "float")
                lg_state.add_variable("stateEstimate.roll",  "float")
                lg_state.add_variable("stateEstimate.pitch", "float")
                lg_state.add_variable("stateEstimate.yaw",   "float")

                # log group 2 - battery
                lg_power = LogConfig(name = "Power", period_in_ms = LOG_PERIOD_MS)
                lg_power.add_variable("pm.vbat", "float")

                # log group 3 - motor outputs
                lg_motors = LogConfig(name = "Motors", period_in_ms = LOG_PERIOD_MS)
                lg_motors.add_variable("motor.m1", "uint16_t")
                lg_motors.add_variable("motor.m2", "uint16_t")
                lg_motors.add_variable("motor.m3", "uint16_t")
                lg_motors.add_variable("motor.m4", "uint16_t")

                # shared partial-state accumulator; logs from the 3 groups arrive asynchronously
                _p    = {"x": 0., "y": 0., "z": 0., "roll": 0., "pitch": 0., "yaw": 0.,
                         "vbat": 0., "m1": 0., "m2": 0., "m3": 0., "m4": 0.}
                _lock = threading.Lock()

                def _flush():
                    with _lock:
                        p = _p.copy()
                    self.state.update(
                        [p["x"], p["y"], p["z"]],
                        _euler_zyx_to_R(p["roll"], p["pitch"], p["yaw"]),
                        p["roll"], p["pitch"], p["yaw"],
                        p["vbat"],
                        [p["m1"], p["m2"], p["m3"], p["m4"]],
                    )

                def _cb_state(timestamp, data, logconf):
                    with _lock:
                        _p.update({"x":     data["stateEstimate.x"],
                                   "y":     data["stateEstimate.y"],
                                   "z":     data["stateEstimate.z"],
                                   "roll":  data["stateEstimate.roll"],
                                   "pitch": data["stateEstimate.pitch"],
                                   "yaw":   data["stateEstimate.yaw"]})
                    _flush()

                def _cb_power(timestamp, data, logconf):
                    with _lock:
                        _p["vbat"] = data["pm.vbat"]
                    _flush()

                def _cb_motors(timestamp, data, logconf):
                    with _lock:
                        _p.update({"m1": data["motor.m1"], "m2": data["motor.m2"],
                                   "m3": data["motor.m3"], "m4": data["motor.m4"]})
                    _flush()

                scf.cf.log.add_config(lg_state)
                scf.cf.log.add_config(lg_power)
                scf.cf.log.add_config(lg_motors)
                lg_state.data_received_cb.add_callback(_cb_state)
                lg_power.data_received_cb.add_callback(_cb_power)
                lg_motors.data_received_cb.add_callback(_cb_motors)
                lg_state.start()
                lg_power.start()
                lg_motors.start()

                while not self._stop.is_set():
                    time.sleep(0.05)

                lg_state.stop()
                lg_power.stop()
                lg_motors.stop()

        except Exception as exc:
            print(f"[CFLIB] Connection error ({self.uri}): {exc}")
        finally:
            self._cf = None
            self.state.connected = False

    def stop(self):
        self._stop.set()

    def blink_led(self, n_blinks = 3, period = 1.0):
        """
        Blink the onboard LED n_blinks times over n_blinks * period seconds (1 blink/period).
        Uses the led.bitmask parameter: 255 = all on, 0 = all off.
        """
        cf = self._cf
        if cf is None or not self.state.connected:
            print(f"[CFLIB] Cannot blink LED - drone not connected ({self.uri})")
            return

        def _do_blink():
            for _ in range(n_blinks):
                try:
                    cf.param.set_value("led.bitmask", str(255))
                    time.sleep(period * 0.5)
                    cf.param.set_value("led.bitmask", str(0))
                    time.sleep(period * 0.5)
                except Exception as exc:
                    print(f"[CFLIB] LED blink error: {exc}")
                    break

        threading.Thread(target = _do_blink, daemon = True, name = "led-blink").start()
