#!/usr/bin/env python3
"""
Xbox Controller Input Module
Polls a connected Xbox (or compatible) gamepad via pygame.joystick and dispatches
flight commands to the Crazyflie fleet.

Button / axis layout assumed (Xbox One / 360 via SDL2):
    Axes:
        0  - Left  stick X   (left = -1, right = +1)
        1  - Left  stick Y   (up   = -1, down  = +1)   ← SDL Y-axis inverted
        2  - Left  trigger   LT  (released = -1, pressed = +1)
        3  - Right stick X   (left = -1, right = +1)
        4  - Right stick Y   (up   = -1, down  = +1)   ← SDL Y-axis inverted
        5  - Right trigger   RT  (released = -1, pressed = +1)

    Buttons:
        0  - A
        1  - B
        2  - X   (unused by mapping)
        3  - Y   (unused by mapping)
        4  - LB  (L1)
        5  - RB  (R1)
        6  - Back / Select
        7  - Start
        8  - Mode / Guide  (may not be exposed by all OS drivers)
        9  - Left  stick click  (unused)
        10 - Right stick click  (unused)

    D-pad / Hat:
        hat(0) → (x, y)  where x ∈ {-1,0,+1}, y ∈ {-1,0,+1}
        up=(0,1), down=(0,-1), left=(-1,0), right=(1,0)

Notes
-----
- All flight commands are sent to the currently selected drone (self.controlled_num).
  When controlled_num == 0 all connected drones receive the command.
- Button 8 (Guide/Mode) may not appear on Linux without special udev rules.
  On Linux the mode button index may fall back to index 8 or may not be detected;
  we detect it through both a direct button press and a synthetic fallback via
  the JOYDEVICEADDED event re-scan.
- The L2 / RT axes are normalised from [-1, +1] to [0, 1] before use.
"""

import math
import time
import threading
import numpy as np

try:
    import pygame
    PYGAME_AVAILABLE = True
except ImportError:
    PYGAME_AVAILABLE = False


# ==============================================================================
# CONFIGURATION
# ==============================================================================

DEADZONE          = 0.12    # joystick axis deadzone (absolute value)
POLL_HZ           = 50      # controller-polling loop frequency
MAX_THRUST        = 65535   # maximum motor PWM
MAX_YAW_RATE      = 360.0   # degrees/sec at full deflection
MAX_ATTITUDE_DEG  = 25.0    # max roll/pitch in degrees

# Movement step sizes for discrete d-pad commands (auto mode)
STEP_LARGE_M      = 0.5     # metres (large step, default)
STEP_SMALL_M      = 0.1     # metres (small step, A button held)
STEP_LARGE_DEG    = 15.0    # degrees yaw (large step)
STEP_SMALL_DEG    = 5.0     # degrees yaw (small step)

# Auto-mode hover hold: vertical velocity when using sticks
ZDOT_RATE         = 0.4     # m/s per unit stick deflection

# Manual mode: baseline hover thrust (send when sticks are centred)
HOVER_THRUST_BASE = 35000   # approximate hover thrust for Crazyflie 2.x


# ==============================================================================
# XBOX CONTROLLER
# ==============================================================================

class XboxController:
    """
    Background-thread Xbox gamepad poller.

    Parameters
    ----------
    drones : dict[int, DroneEntry]
        Shared fleet dict from visualization.py (read/written in the main thread;
        access is safe for reading because Python GIL protects simple dict reads,
        and the controller thread only reads the dict, never modifies it).

    Public attributes (read by visualization.py for the HUD):
        mode           : "manual" | "auto"
        controlled_num : int - currently selected drone number (0 = all)
    """

    # button index constants
    BTN_A      = 0
    BTN_B      = 1
    BTN_LB     = 4
    BTN_RB     = 5
    BTN_SELECT = 6
    BTN_START  = 7
    BTN_MODE   = 8   # Guide button - may not be exposed on all platforms

    # axis index constants
    AXIS_LX    = 0   # left stick  X
    AXIS_LY    = 1   # left stick  Y (inverted: up = -1)
    AXIS_LT    = 2   # left  trigger
    AXIS_RX    = 3   # right stick X
    AXIS_RY    = 4   # right stick Y (inverted: up = -1)
    AXIS_RT    = 5   # right trigger

    def __init__(self, drones):
        self._drones         = drones
        self.mode            = "auto"
        self.controlled_num  = 0       # 0 = all drones
        self._joystick       = None
        self._stop_ev        = threading.Event()
        self._thread         = None

        # drone-selection state (L1 / R1 + Select workflow)
        self._pending_num    = None    # tentative number during L1/R1 cycling
        self._lb_held        = False
        self._rb_held        = False

        # takeoff/land toggle (Start button in auto mode)
        self._airborne       = False   # tracks whether the drone is considered in flight

        # step-size modifier (A = small, B = large; A takes priority)
        self._a_held         = False
        self._b_held         = False

        # last sent setpoint (avoid flooding)
        self._last_cmd_t     = 0.0
        self._cmd_interval   = 1.0 / POLL_HZ

        # last discrete command string, read by visualization.py for the toast overlay
        # set to (text, timestamp) whenever a notable action is triggered; None otherwise
        self.last_cmd        = None   # (str, float) | None

    # public interface

    def start(self):
        """Start the background polling thread."""
        self._stop_ev.clear()
        self._thread = threading.Thread(target = self._run, daemon = True,
                                        name = "xbox-poll")
        self._thread.start()

    def stop(self):
        """Signal the polling thread to stop and wait for it."""
        self._stop_ev.set()
        if self._thread is not None:
            self._thread.join(timeout = 1.0)

    def is_connected(self):
        """Return True if a joystick is currently active."""
        return self._joystick is not None

    def controller_name(self):
        """Return the SDL name string of the connected joystick, or empty string."""
        if self._joystick is not None:
            try:
                return self._joystick.get_name()
            except Exception:
                pass
        return ""

    # internal helpers

    def _axis(self, idx):
        """Return a deadzone-applied axis value in [-1, +1]."""
        try:
            v = self._joystick.get_axis(idx)
        except Exception:
            return 0.0
        return v if abs(v) > DEADZONE else 0.0

    def _trigger(self, idx):
        """
        Return a trigger value in [0, 1].
        On Linux/SDL2 (Xbox One driver) triggers rest at -1, fully pressed = +1.
        On Windows (xinput / Xbox 360) triggers rest at 0, fully pressed = +1.
        max(0, raw) maps both cases correctly without division artefacts.
        """
        try:
            raw = self._joystick.get_axis(idx)
        except Exception:
            return 0.0
        return max(0.0, float(raw))

    def _btn(self, idx):
        """Return True if button idx is pressed; False if the index doesn't exist."""
        try:
            if self._joystick is None or idx >= self._joystick.get_numbuttons():
                return False
            return bool(self._joystick.get_button(idx))
        except Exception:
            return False

    def _hat(self):
        """Return the d-pad (hat) state as (x, y) tuple."""
        try:
            return self._joystick.get_hat(0)
        except Exception:
            return (0, 0)

    def _step(self):
        """Return the current step sizes (m, deg) based on A/B held state."""
        if self._a_held:
            return STEP_SMALL_M, STEP_SMALL_DEG
        return STEP_LARGE_M, STEP_LARGE_DEG

    def _targets(self):
        """Return list of DroneEntry objects to command."""
        n = self.controlled_num
        if n == 0:
            return list(self._drones.values())
        return [self._drones[n]] if n in self._drones else []

    def _thread_for(self, entry):
        """Return the CrazyflieThread for an entry, or None if not connected."""
        t = entry.thread
        if t is None or not entry.state.connected:
            return None
        return t

    # flight commands

    def _cmd_takeoff(self, height = 0.5):
        for e in self._targets():
            t = self._thread_for(e)
            if t: t.takeoff(height)
        self._airborne = True

    def _cmd_land(self):
        for e in self._targets():
            t = self._thread_for(e)
            if t: t.land()
        self._airborne = False

    def _cmd_emergency_stop(self):
        for e in self._targets():
            t = self._thread_for(e)
            if t: t.emergency_stop()
        self._airborne = False

    def _cmd_go_to(self, dx = 0., dy = 0., dz = 0., dyaw_deg = 0.):
        for e in self._targets():
            t = self._thread_for(e)
            if t:
                # rotate dx/dy from world frame to account for current yaw
                _, _, _, _, yaw, _, _ = e.state.get()
                yr = math.radians(yaw)
                wx = dx * math.cos(yr) - dy * math.sin(yr)
                wy = dx * math.sin(yr) + dy * math.cos(yr)
                t.go_to(dx = wx, dy = wy, dz = dz, dyaw_deg = dyaw_deg)

    def _cmd_setpoint_manual(self, roll, pitch, yawrate, thrust):
        for e in self._targets():
            t = self._thread_for(e)
            if t: t.send_setpoint(roll, pitch, yawrate, thrust)

    def _cmd_hover(self, vx, vy, yawrate, zdot):
        for e in self._targets():
            t = self._thread_for(e)
            if t: t.send_hover_setpoint(vx, vy, yawrate, zdot)

    def _cmd_blink(self):
        for e in self._targets():
            t = self._thread_for(e)
            if t: t.blink_led()

    # polling loop

    def _try_init_joystick(self):
        """Attempt to initialise the first available joystick."""
        if not PYGAME_AVAILABLE:
            return
        try:
            # Do NOT call pygame.joystick.quit() here — it tears down SDL's internal
            # device-ID map while the main thread may be inside pygame.event.get(),
            # causing a KeyError that propagates as a SystemError. joystick.init() is
            # idempotent when the subsystem is already running.
            pygame.joystick.init()
            if pygame.joystick.get_count() > 0:
                joy = pygame.joystick.Joystick(0)
                joy.init()
                self._joystick = joy
                print(f"[Xbox] Connected: {joy.get_name()}")
        except Exception as exc:
            print(f"[Xbox] Joystick init error: {exc}")
            self._joystick = None

    def _run(self):
        self._try_init_joystick()

        # per-frame state for edge-detection (buttons we only act on rising edge)
        _prev_btn = {}

        def rising(idx):
            cur = self._btn(idx)
            prev = _prev_btn.get(idx, False)
            _prev_btn[idx] = cur
            return cur and not prev

        def falling(idx):
            cur = self._btn(idx)
            prev = _prev_btn.get(idx, False)
            _prev_btn[idx] = cur
            return not cur and prev

        _prev_hat = (0, 0)

        while not self._stop_ev.is_set():
            t0 = time.monotonic()

            # re-scan for joystick if lost
            if self._joystick is None:
                self._try_init_joystick()
                if self._joystick is None:
                    time.sleep(2.0)
                    continue

            # NOTE: do NOT call pygame.event.pump() here — the main thread's
            # pygame.event.get() already pumps the event queue. Calling pump()
            # from a background thread causes a SystemError / race condition.

            # verify joystick is still alive
            try:
                _ = self._joystick.get_name()
            except Exception:
                print("[Xbox] Joystick disconnected.")
                self._joystick = None
                continue

            # read raw inputs 

            lx   = self._axis(self.AXIS_LX)    # left stick X:  left=-1, right=+1
            ly   = -self._axis(self.AXIS_LY)   # left stick Y:  up=+1  (invert SDL)
            lt   = self._trigger(self.AXIS_LT) # left trigger:  0..1
            rx   = self._axis(self.AXIS_RX)    # right stick X: left=-1, right=+1
            ry   = -self._axis(self.AXIS_RY)   # right stick Y: up=+1  (invert SDL)
            rt   = self._trigger(self.AXIS_RT) # right trigger: 0..1
            hat  = self._hat()                 # d-pad (x, y)

            # step modifier buttons (hold)
            self._a_held = self._btn(self.BTN_A)
            self._b_held = self._btn(self.BTN_B)
            step_m, step_deg = self._step()

            # L2 emergency stop (rising edge of trigger crossing 0.5)
            if lt > 0.5 and _prev_btn.get("lt_half", False) is False:
                _prev_btn["lt_half"] = True
                self._cmd_emergency_stop()
                self.last_cmd = ("EMERGENCY STOP |||", time.monotonic())
                print("[Xbox] Emergency stop!")
            elif lt <= 0.5:
                _prev_btn["lt_half"] = False

            # R2 blink LED (rising edge)
            if rt > 0.5 and _prev_btn.get("rt_half", False) is False:
                _prev_btn["rt_half"] = True
                self._cmd_blink()
                self.last_cmd = ("Blink LED", time.monotonic())
                print("[Xbox] Blink LED!")
            elif rt <= 0.5:
                _prev_btn["rt_half"] = False

            # drone-selection workflow (L1 / R1 cycle, Select confirm)
            lb_now = self._btn(self.BTN_LB)
            rb_now = self._btn(self.BTN_RB)

            if rising(self.BTN_LB):
                # start cycling or go lower
                if self._pending_num is None:
                    self._pending_num = self.controlled_num
                nums = sorted([0] + list(self._drones.keys()))
                idx  = nums.index(self._pending_num) if self._pending_num in nums else 0
                self._pending_num = nums[(idx - 1) % len(nums)]
                self.last_cmd = (f"Selecting CF {self._pending_num} (press Select to confirm)", time.monotonic())
                print(f"[Xbox] Selecting CF {self._pending_num} (press Select to confirm)")

            if rising(self.BTN_RB):
                if self._pending_num is None:
                    self._pending_num = self.controlled_num
                nums = sorted([0] + list(self._drones.keys()))
                idx  = nums.index(self._pending_num) if self._pending_num in nums else 0
                self._pending_num = nums[(idx + 1) % len(nums)]
                self.last_cmd = (f"Selecting CF {self._pending_num} (press Select to confirm)", time.monotonic())
                print(f"[Xbox] Selecting CF {self._pending_num} (press Select to confirm)")

            if rising(self.BTN_SHARE):
                # The Xbox Guide/Mode button (BTN_MODE = 8) is intercepted by Windows
                # for the Game Bar and cannot be received by applications.  We use the
                # Back/Select button (BTN_SELECT = 6) instead when it is pressed outside
                # of a drone-selection workflow (no conflict — Select only acts on a
                # pending number started by LB/RB).
                if self._pending_num is None:
                    self.mode = "auto" if self.mode == "manual" else "manual"
                    print(f"[Xbox] Mode → {self.mode.upper()}")
                    self.last_cmd = (f"Mode → {self.mode.upper()}", time.monotonic())
                    # put previously controlled drone into auto hold when switching to auto
                    if self.mode == "auto":
                        for e in self._targets():
                            t = self._thread_for(e)
                            if t: t.send_hover_setpoint(0., 0., 0., 0.)
                else:
                    # new drone selection to control
                    # put old controlled drone into auto hover before switching
                    if self.mode == "auto":
                        for e in self._targets():
                            t = self._thread_for(e)
                            if t: t.send_hover_setpoint(0., 0., 0., 0.)
                    self.controlled_num = self._pending_num
                    self._pending_num   = None
                    self.last_cmd = (f"Now controlling CF {self.controlled_num} !", time.monotonic())
                    print(f"[Xbox] Now controlling CF {self.controlled_num}")

            # cancel pending selection if any other significant input is given
            if self._pending_num is not None:
                any_sig = (abs(lx) > DEADZONE or abs(ly) > DEADZONE or
                           abs(rx) > DEADZONE or abs(ry) > DEADZONE or
                           hat != (0, 0) or self._btn(self.BTN_START))
                if any_sig:
                    self._pending_num = None
                    self.last_cmd = (f"Selection cancelled - staying on CF {self.controlled_num} !", time.monotonic())
                    print(f"[Xbox] Selection cancelled - staying on CF {self.controlled_num}")

            # Start button: takeoff / land toggle (auto mode only)
            if rising(self.BTN_START) and self.mode == "auto":
                if not self._airborne:
                    self._cmd_takeoff(STEP_LARGE_M)
                    self.last_cmd = (f"Takeoff → {STEP_LARGE_M:.3f} m !", time.monotonic())
                    print(f"[Xbox] Takeoff → {STEP_LARGE_M:.3f} m !")
                else:
                    self._cmd_land()
                    self.last_cmd = ("Land !", time.monotonic())
                    print(f"[Xbox] Land !")

            # D-pad: discrete position / altitude steps (auto mode only)
            hat_edge = hat != _prev_hat
            _prev_hat = hat
            if hat_edge and self.mode == "auto":
                hx, hy = hat
                if   hy == +1:
                    self._cmd_go_to(dz = +step_m)
                    self.last_cmd = (f"↑ Up +{step_m:.3f} m", time.monotonic())
                elif hy == -1:
                    self._cmd_go_to(dz = -step_m)
                    self.last_cmd = (f"↓ Down -{step_m:.2f} m", time.monotonic())
                elif hx == +1:
                    self._cmd_go_to(dy = -step_m)
                    self.last_cmd = (f"→ Slide Right +{step_m:.2f} m", time.monotonic())
                elif hx == -1:
                    self._cmd_go_to(dy = +step_m)
                    self.last_cmd = (f"← Slide Left -{step_m:.2f} m", time.monotonic())

            # continuous stick commands
            now = time.monotonic()
            if now - self._last_cmd_t >= self._cmd_interval:
                self._last_cmd_t = now

                if self.mode == "manual":
                    # left stick:  ly = thrust (up = more),  lx = yaw rate
                    # right stick: rx = roll,               ry = pitch
                    # Only send if the left stick is pushed up (ly > 0) so that
                    # motors stay off when the controller is at rest.
                    if ly > DEADZONE:
                        thrust  = int(HOVER_THRUST_BASE + ly * (MAX_THRUST - HOVER_THRUST_BASE))
                        thrust  = max(0, min(MAX_THRUST, thrust))
                        yawrate = -lx * MAX_YAW_RATE
                        roll    =  rx * MAX_ATTITUDE_DEG
                        pitch   =  ry * MAX_ATTITUDE_DEG
                        self._cmd_setpoint_manual(roll, pitch, yawrate, thrust)
                        self.last_cmd = (
                            f"Manual: roll {roll:+.2f}°, pitch {pitch:+.2f}°, "
                            f"yaw {yawrate:+.2f}°/s, thrust {thrust}",
                            time.monotonic()
                        )
                        print(
                            f"[Xbox] Manual: roll {roll:+.2f}°, pitch {pitch:+.2f}°, "
                            f"yaw {yawrate:+.2f}°/s, thrust {thrust}"
                        )
                    else:
                        # stick not raised - send zero thrust to keep motors off
                        self._cmd_setpoint_manual(0., 0., 0., 0)
                        
                else:  # auto (velocity hover)
                    # left stick:  ly = zdot (up/down),  lx = yaw rate
                    # right stick: ry = forward/back,    rx = slide left/right
                    # The deadzone is large enough to absorb rest-position drift,
                    # so we send whenever any stick is outside it.
                    sticks_active = (abs(lx) > DEADZONE or abs(ly) > DEADZONE or
                                    abs(rx) > DEADZONE or abs(ry) > DEADZONE)

                    if sticks_active:
                        self._cmd_hover(vx, vy, yawrate, zdot)
                        zdot    =  ly * ZDOT_RATE
                        yawrate = -lx * MAX_YAW_RATE
                        vx      =  ry  # forward positive in body frame
                        vy      = -rx  # left positive in body frame
                        self.last_cmd = (
                            f"Auto hover: vx {vx:+.3f}, vy {vy:+.3f}, "
                            f"zdot {zdot:+.3f}, yaw {yawrate:+.2f}°/s",
                            time.monotonic()
                        )
                        print(
                            f"[Xbox] Auto hover: vx {vx:+.3f}, vy {vy:+.3f}, "
                            f"zdot {zdot:+.3f}, yaw {yawrate:+.2f}°/s"
                        )

            # maintain loop frequency
            elapsed = time.monotonic() - t0
            sleep_t = max(0., (1.0 / POLL_HZ) - elapsed)
            time.sleep(sleep_t)
