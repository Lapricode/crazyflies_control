#!/usr/bin/env python3
"""
PlayStation Controller Input Module
Polls a connected PlayStation (DS4 / DS5 or compatible) gamepad via pygame.joystick
and dispatches flight commands to the Crazyflie fleet.

Button / axis layout assumed (PS4 DS4 / PS5 DS5 via SDL2):
    Axes:
        0  - Left  stick X   (left = -1, right = +1)
        1  - Left  stick Y   (up   = -1, down  = +1)   ← SDL Y-axis inverted
        2  - Right stick X   (left = -1, right = +1)
        3  - Right stick Y   (up   = -1, down  = +1)   ← SDL Y-axis inverted
        4  - L2 trigger      (released = -1 on Linux / 0 on Windows, pressed = +1)
        5  - R2 trigger      (released = -1 on Linux / 0 on Windows, pressed = +1)

    Buttons:
        0  - Cross    (x-A)
        1  - Circle   (○-B)
        2  - Square   (□-X)
        3  - Triangle (△-Y)
        4  - L1
        5  - R1
        6  - Share  (DS4) / Create (DS5)
        7  - Options
        8  - PS button  (may be intercepted by the OS)
        9  - L3  (left  stick click)  (unused)
        10 - R3  (right stick click)  (unused)
        11 - Touchpad click           (unused, DS4/DS5 only)

    D-pad / Hat:
        hat(0) → (x, y)  where x ∈ {-1,0,+1}, y ∈ {-1,0,+1}
        up=(0,1), down=(0,-1), left=(-1,0), right=(1,0)

Notes
-----
- All flight commands are sent to the currently selected drone (self.controlled_num).
  When controlled_num == 0 all connected drones receive the command.
- The PS button (index 8) is typically captured by the OS (PS overlay / Steam).
  Mode toggle is therefore mapped to Share (DS4) / Create (DS5).
- L2 / R2 axes are normalised to [0, 1] via max(0, raw), which works for both
  Linux (rest = -1) and Windows (rest = 0) driver behaviours.
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

DEADZONE          = 0.10    # joystick axis deadzone (absolute value)
POLL_HZ           = 50      # controller-polling loop frequency
MAX_THRUST        = 65535   # maximum motor PWM
MAX_YAW_RATE      = 360.0   # degrees/sec at full deflection
MAX_ATTITUDE_DEG  = 25.0    # max roll/pitch in degrees

# Movement step sizes for discrete d-pad commands (auto mode)
STEP_LARGE_M      = 0.5     # metres (large step, default)
STEP_SMALL_M      = 0.1     # metres (small step, Cross button held)
STEP_LARGE_DEG    = 15.0    # degrees yaw (large step)
STEP_SMALL_DEG    = 5.0     # degrees yaw (small step)

# Auto-mode hover hold: vertical velocity when using sticks
ZDOT_RATE         = 0.4     # m/s per unit stick deflection

# Manual mode: baseline hover thrust (send when sticks are centred)
HOVER_THRUST_BASE = 20000   # approximate hover thrust for Crazyflie 2.x


# ==============================================================================
# PLAYSTATION CONTROLLER
# ==============================================================================

class PlaystationController:
    """
    Background-thread PlayStation gamepad poller (DS4 / DS5 or compatible).

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

    # button index constants  (SDL2 DS4/DS5 layout)
    BTN_CROSS    = 0   # x-A  - step-size: small
    BTN_CIRCLE   = 1   # ○-B  - step-size: large
    BTN_SQUARE   = 2   # □-X  - reconnect with selected drones
    BTN_TRIANGLE = 3   # △-Y  - enable first person camera
    BTN_L1       = 4
    BTN_R1       = 5
    BTN_SHARE    = 6   # Share (DS4) / Create (DS5) — mode toggle / drone select confirm
    BTN_OPTIONS  = 7   # Options — takeoff / land
    BTN_PS       = 8   # PS button — usually captured by OS

    # axis index constants  (SDL2 DS4/DS5 layout)
    AXIS_LX    = 0   # left  stick X
    AXIS_LY    = 1   # left  stick Y (inverted: up = -1)
    AXIS_RX    = 2   # right stick X   ← PS: index 2 (Xbox: index 3)
    AXIS_RY    = 3   # right stick Y (inverted: up = -1)   ← PS: index 3 (Xbox: index 4)
    AXIS_L2    = 4   # L2 trigger   ← PS: index 4 (Xbox LT: index 2)
    AXIS_R2    = 5   # R2 trigger   ← PS: index 5 (same as Xbox RT)

    def __init__(self, drones, set_first_person_camera = None, reset_camera_tracking = None, is_first_person_camera = None):
        self._drones         = drones
        self.mode            = "auto"
        self.controlled_num  = 0       # 0 = all drones
        self._joystick       = None
        self._stop_ev        = threading.Event()
        self._thread         = None

        # drone-selection state (L1 / R1 + Share workflow)
        self._pending_num    = None    # tentative number during L1/R1 cycling
        self._l1_held        = False
        self._r1_held        = False

        # takeoff/land toggle (Options button in auto mode)
        self._airborne       = False   # tracks whether the drone is considered in flight

        # step-size modifier (Cross = small, Circle = large; Cross takes priority)
        self._cross_held     = False
        self._circle_held    = False

        # last sent setpoint (avoid flooding)
        self._last_cmd_t     = 0.0
        self._cmd_interval   = 1.0 / POLL_HZ

        # set to True by visualization.py when the keyboard flight panel is open;
        # suppresses all gamepad flight commands to avoid conflicts
        self.panel_open      = False

        # last discrete command string, read by visualization.py for the toast overlay
        self.last_cmd        = None   # (str, float) | None

        # camera hooks
        self._set_first_person_camera = set_first_person_camera
        self._reset_camera_tracking   = reset_camera_tracking
        self._is_first_person_camera  = is_first_person_camera
        self._fp_camera_enabled       = False

    # public interface

    def start(self):
        """Start the background polling thread."""
        self._stop_ev.clear()
        self._thread = threading.Thread(target = self._run, daemon = True,
                                        name = "ps-poll")
        self._thread.start()

    def stop(self):
        """Signal the polling thread to stop and wait for it."""
        self._stop_ev.set()
        if self._thread is not None:
            self._thread.join(timeout = 1.0)

    def is_connected(self):
        """Return True if a joystick is currently active."""
        return self._joystick is not None

    def force_disconnect(self):
        """
        Called from the main thread when a JOYDEVICEREMOVED event is received.
        Clears the joystick handle so the HUD immediately shows 'not connected'
        and the poll loop re-scans on its next iteration.
        """
        self._release_control()
        self._joystick = None
        self._airborne = False   # reset flight state on hardware disconnect
        print("[PS] Joystick removed (forced disconnect).")

    def controller_name(self):
        """Return the SDL name string of the connected joystick, or empty string."""
        if self._joystick is not None:
            try:
                return self._joystick.get_name()
            except Exception:
                pass
        return ""

    # internal helpers

    def _camera_target_num(self):
        """
        Prefer the currently selected drone.
        If controlled_num == 0, use the first available connected/known drone.
        """
        if self.controlled_num != 0 and self.controlled_num in self._drones:
            return self.controlled_num
        nums = sorted(self._drones.keys())
        return nums[0] if nums else None

    def _camera_is_first_person(self):
        """
        Sync to visualization.py if a status callback was provided.
        Otherwise fall back to the controller's local toggle state.
        """
        if callable(self._is_first_person_camera):
            try:
                return bool(self._is_first_person_camera())
            except Exception:
                pass
        return self._fp_camera_enabled

    def _cmd_reconnect_selected(self):
        """
        Reconnect the currently targeted drone(s).
        Uses DroneEntry.start_connection() from visualization.py.
        """
        targets = self._targets()
        if not targets:
            self.last_cmd = ("Reconnect: no selected drone", time.monotonic())
            print("[PS] Reconnect skipped: no selected drone.")
            return

        self._release_control()
        for e in targets:
            try:
                e.start_connection()
            except Exception as exc:
                print(f"[PS] Reconnect failed for CF {e.num}: {exc}")

        self._airborne = False
        if self.controlled_num == 0:
            self.last_cmd = ("Reconnect selected drones: all CF", time.monotonic())
        else:
            self.last_cmd = (f"Reconnect selected drones: CF {self.controlled_num}", time.monotonic())

    def _cmd_first_person_camera(self):
        n = self._camera_target_num()
        if n is None:
            self.last_cmd = ("Camera: no drone available", time.monotonic())
            print("[PS] No drone available for first-person camera.")
            return

        if callable(self._set_first_person_camera):
            self._set_first_person_camera(n)

        self._fp_camera_enabled = True
        self.last_cmd = (f"Camera → first-person CF {n}", time.monotonic())
        print(f"[PS] Camera → first-person CF {n}")

    def _cmd_normal_camera(self):
        if callable(self._reset_camera_tracking):
            self._reset_camera_tracking()

        self._fp_camera_enabled = False
        self.last_cmd = ("Camera → normal view", time.monotonic())
        print("[PS] Camera → normal view")

    def _release_control(self):
        for e in self._targets():
            t = self._thread_for(e)
            if t:
                t.stop_setpoint_stream()
                
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
        On Linux/SDL2 (DS4/DS5 hidraw) triggers rest at -1, fully pressed = +1.
        On Windows (DS4Windows / native HID) triggers rest at 0, fully pressed = +1.
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
        """Return the current step sizes (m, deg) based on Cross/Circle held state."""
        if self._cross_held:
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
                print(f"[PS] Connected: {joy.get_name()}")
        except Exception as exc:
            print(f"[PS] Joystick init error: {exc}")
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
                print("[PS] Joystick disconnected.")
                self._joystick = None
                continue

            # read raw inputs

            lx   = self._axis(self.AXIS_LX)    # left  stick X: left=-1, right=+1
            ly   = -self._axis(self.AXIS_LY)   # left  stick Y: up=+1  (invert SDL)
            lt   = self._trigger(self.AXIS_L2) # L2 trigger:    0..1
            rx   = self._axis(self.AXIS_RX)    # right stick X: left=-1, right=+1
            ry   = -self._axis(self.AXIS_RY)   # right stick Y: up=+1  (invert SDL)
            rt   = self._trigger(self.AXIS_R2) # R2 trigger:    0..1
            hat  = self._hat()                 # d-pad (x, y)

            # step modifier buttons (hold)
            self._cross_held  = self._btn(self.BTN_CROSS)
            self._circle_held = self._btn(self.BTN_CIRCLE)
            step_m, step_deg  = self._step()

            # ── L2 emergency stop (rising edge of trigger crossing 0.5) ────────
            if lt > 0.5 and _prev_btn.get("lt_half", False) is False:
                _prev_btn["lt_half"] = True
                self._cmd_emergency_stop()
                self.last_cmd = ("EMERGENCY STOP !!!", time.monotonic())
                print("[PS] Emergency stop!")
            elif lt <= 0.5:
                _prev_btn["lt_half"] = False

            # ── R2 blink LED (rising edge) ─────────────────────────────────────
            if rt > 0.5 and _prev_btn.get("rt_half", False) is False:
                _prev_btn["rt_half"] = True
                self._cmd_blink()
                self.last_cmd = ("Blink LED", time.monotonic())
                print("[PS] Blink LED!")
            elif rt <= 0.5:
                _prev_btn["rt_half"] = False

            # skip all other flight commands when keyboard panel is open
            if self.panel_open:
                elapsed = time.monotonic() - t0
                time.sleep(max(0., (1.0 / POLL_HZ) - elapsed))
                continue

            # ── drone-selection workflow (L1 / R1 cycle, Share confirm) ────────
            lb_now = self._btn(self.BTN_L1)
            rb_now = self._btn(self.BTN_R1)

            if rising(self.BTN_L1):
                # start cycling or go lower
                if self._pending_num is None:
                    self._pending_num = self.controlled_num
                nums = sorted([0] + list(self._drones.keys()))
                idx  = nums.index(self._pending_num) if self._pending_num in nums else 0
                self._pending_num = nums[(idx - 1) % len(nums)]
                if self._pending_num == 0:  # select all current drones
                    self.last_cmd = (f"Selecting all CF (press Select to confirm)", time.monotonic())
                    print(f"[PS] Selecting all CF (press Select to confirm)")
                else:
                    self.last_cmd = (f"Selecting CF {self._pending_num} (press Select to confirm)", time.monotonic())
                    print(f"[PS] Selecting CF {self._pending_num} (press Select to confirm)")

            if rising(self.BTN_R1):
                if self._pending_num is None:
                    self._pending_num = self.controlled_num
                nums = sorted([0] + list(self._drones.keys()))
                idx  = nums.index(self._pending_num) if self._pending_num in nums else 0
                self._pending_num = nums[(idx + 1) % len(nums)]
                if self._pending_num == 0:  # select all current drones
                    self.last_cmd = (f"Selecting all CF (press Select to confirm)", time.monotonic())
                    print(f"[PS] Selecting all CF (press Select to confirm)")
                else:
                    self.last_cmd = (f"Selecting CF {self._pending_num} (press Select to confirm)", time.monotonic())
                    print(f"[PS] Selecting CF {self._pending_num} (press Select to confirm)")

            if rising(self.BTN_SHARE):
                # ── mode toggle ────────────────────────────────────────────────────
                # The PS button (BTN_PS = 8) is intercepted by the OS (PS overlay / Steam)
                # and cannot reliably be received by applications.  Share (DS4) / Create
                # (DS5) is used instead when pressed outside a drone-selection workflow
                # (no conflict — Share only acts on a pending number started by L1/R1).
                if self._pending_num is None:
                    self._release_control()
                    self.mode = "auto" if self.mode == "manual" else "manual"
                    print(f"[PS] Mode → {self.mode.upper()}")
                    self.last_cmd = (f"Mode → {self.mode.upper()}", time.monotonic())
                    # put previously controlled drone into auto hold when switching to auto
                    if self.mode == "auto":
                        self._airborne = False
                        self._auto_resume_at = time.monotonic() + 0.5
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
                    if self.controlled_num == 0:  # select all current drones
                        self.last_cmd = (f"Now controlling all CF !", time.monotonic())
                        print(f"[PS] Now controlling all CF !")
                    else:
                        self.last_cmd = (f"Now controlling CF {self.controlled_num} !", time.monotonic())
                        print(f"[PS] Now controlling CF {self.controlled_num} !")
                    self._cmd_blink()

            # reconnect selected drones
            if rising(self.BTN_SQUARE):
                self._cmd_reconnect_selected()
                print("[PS] Reconnect selected drones")

            # toggle first-person camera
            if rising(self.BTN_TRIANGLE):
                if self._camera_is_first_person():
                    self._cmd_normal_camera()
                    print("[PS] Camera → normal view")
                else:
                    self._cmd_first_person_camera()
                    print("[PS] Camera → first-person")

            # cancel pending selection if any other significant input is given
            if self._pending_num is not None:
                any_sig = (abs(lx) > DEADZONE or abs(ly) > DEADZONE or
                           abs(rx) > DEADZONE or abs(ry) > DEADZONE or
                           hat != (0, 0) or self._btn(self.BTN_OPTIONS))
                if any_sig:
                    self._pending_num = None
                    self.last_cmd = (f"Selection cancelled - staying on CF {self.controlled_num} !", time.monotonic())
                    print(f"[PS] Selection cancelled - staying on CF {self.controlled_num}")

            # ── Options button: takeoff / land toggle (auto mode only) ─────────
            if rising(self.BTN_OPTIONS) and self.mode == "auto":
                self._release_control()
                if not self._airborne:
                    self._cmd_takeoff(STEP_LARGE_M)
                    self._airborne = True
                    self._auto_resume_at = time.monotonic() + 2.2   # or duration + margin
                    self.last_cmd = (f"Takeoff → {STEP_LARGE_M:.3f} m !", time.monotonic())
                    print(f"[PS] Takeoff → {STEP_LARGE_M:.3f} m !")
                else:
                    self._cmd_land()
                    self.last_cmd = ("Land !", time.monotonic())
                    print(f"[PS] Land !")

            # D-pad: discrete position / altitude steps (auto mode only)
            hat_edge = hat != _prev_hat
            _prev_hat = hat
            if hat_edge and self.mode == "auto":
                hx, hy = hat
                if   hy == +1:
                    self._cmd_go_to(dx = +step_m)
                    self.last_cmd = (f"↑ Forward +{step_m:.3f} m", time.monotonic())
                elif hy == -1:
                    self._cmd_go_to(dx = -step_m)
                    self.last_cmd = (f"↓ Backward -{step_m:.3f} m", time.monotonic())
                elif hx == +1:
                    self._cmd_go_to(dy = -step_m)
                    self.last_cmd = (f"→ Slide Right +{step_m:.3f} m", time.monotonic())
                elif hx == -1:
                    self._cmd_go_to(dy = +step_m)
                    self.last_cmd = (f"← Slide Left -{step_m:.3f} m", time.monotonic())

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
                        yawrate =  lx * MAX_YAW_RATE
                        roll    =  rx * MAX_ATTITUDE_DEG
                        pitch   =  ry * MAX_ATTITUDE_DEG
                        self._cmd_setpoint_manual(roll, pitch, yawrate, thrust)
                        self.last_cmd = (
                            f"Manual: roll {roll:+.2f}°, pitch {pitch:+.2f}°, "
                            f"yaw {yawrate:+.2f}°/s, thrust {thrust}",
                            time.monotonic()
                        )
                        print(
                            f"[PS] Manual: roll {roll:+.2f}°, pitch {pitch:+.2f}°, "
                            f"yaw {yawrate:+.2f}°/s, thrust {thrust}"
                        )
                    else:
                        # stick not raised - send zero thrust to keep motors off
                        self._cmd_setpoint_manual(0., 0., 0., 0)

                else:  # auto (velocity hover)

                    if self._airborne and time.monotonic() >= self._auto_resume_at:
                        # left stick:  ly = zdot (up/down),  lx = yaw rate
                        # right stick: ry = forward/back,    rx = slide left/right
                        sticks_active = (abs(lx) > DEADZONE or abs(ly) > DEADZONE or
                                        abs(rx) > DEADZONE or abs(ry) > DEADZONE)

                        if sticks_active:
                            zdot    =  ly * ZDOT_RATE
                            yawrate =  lx * MAX_YAW_RATE
                            vx      =  ry  # forward positive in body frame
                            vy      = -rx  # left positive in body frame
                            self._cmd_hover(vx, vy, yawrate, zdot)
                            self.last_cmd = (
                                f"Auto hover: vx {vx:+.3f}, vy {vy:+.3f}, "
                                f"zdot {zdot:+.3f}, yaw {yawrate:+.2f}°/s",
                                time.monotonic()
                            )
                        else:
                            # airborne and sticks at rest: hold position (zero velocity)
                            self._cmd_hover(0., 0., 0., 0.)
                    # else: not airborne → send nothing; let the HLC/firmware idle
                            
            # maintain loop frequency
            elapsed = time.monotonic() - t0
            sleep_t = max(0., (1.0 / POLL_HZ) - elapsed)
            time.sleep(sleep_t)
