#!/usr/bin/env python3
"""
PlayStation Controller Input Module

Auto-mode only.
High-level commands only:
- Start: takeoff / land
- Sticks: discrete forward/backward and slide left/right
- D-pad: up/down and turn left/right
- L2: turbo (duration 1.0s instead of default 2.0s)
- Cross/Circle: step size modifiers
"""

import math
import time
import threading

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

# Movement step sizes for discrete high-level commands
STEP_SMALL_M      = 0.1     # metres (Cross held)
STEP_SMALL_DEG    = 15.0    # degrees yaw (Cross held)

STEP_NORMAL_M     = 0.25    # metres (default)
STEP_NORMAL_DEG   = 45.0    # degrees yaw (default)

STEP_LARGE_M      = 0.5     # metres (Circle held)
STEP_LARGE_DEG    = 90.0    # degrees yaw (Circle held)

DEFAULT_MOVE_DURATION = 2.0
TURBO_MOVE_DURATION   = 1.0


# ==============================================================================
# PLAYSTATION CONTROLLER
# ==============================================================================

class PlaystationController:
    """
    Background-thread PlayStation gamepad poller (DS4 / DS5 or compatible).

    Public attributes (read by visualization.py for the HUD):
        mode           : always "auto"
        controlled_num : int - currently selected drone number (0 = all)
    """

    # button index constants  (SDL2 DS4/DS5 layout)
    BTN_CROSS    = 0   # small step modifier
    BTN_CIRCLE   = 1   # large step modifier
    BTN_SQUARE   = 2   # connect selected drone(s)
    BTN_TRIANGLE = 3   # first-person camera toggle
    BTN_L1       = 4   # select previous drone
    BTN_R1       = 5   # select next drone
    BTN_SHARE    = 6   # confirm drone selection
    BTN_OPTIONS  = 7   # takeoff / land
    BTN_PS       = 8   # usually captured by OS

    # axis index constants  (SDL2 DS4/DS5 layout)
    AXIS_LX    = 0   # left  stick X  → slide left/right
    AXIS_LY    = 1   # left  stick Y  → forward/backward
    AXIS_RX    = 2   # right stick X  → slide left/right
    AXIS_RY    = 3   # right stick Y  → forward/backward
    AXIS_L2    = 4   # turbo modifier
    AXIS_R2    = 5   # blink LED

    def __init__(self, drones, set_first_person_camera=None, reset_camera_tracking=None, is_first_person_camera=None):
        self._drones        = drones
        self.mode           = "auto"
        self.controlled_num  = 0
        self._joystick      = None
        self._stop_ev       = threading.Event()
        self._thread        = None

        # drone-selection state (L1 / R1 + Share workflow)
        self._pending_num   = None

        # takeoff/land toggle
        self._airborne      = False

        # step-size modifier (Cross = small, Circle = large; Cross takes priority)
        self._cross_held    = False
        self._circle_held   = False

        # last sent command time
        self._last_cmd_t    = 0.0
        self._cmd_interval  = 1.0 / POLL_HZ

        # suppress all flight commands when the keyboard panel is open
        self.panel_open     = False

        # last discrete command string, read by visualization.py for the toast overlay
        self.last_cmd       = None  # (str, float) | None

        # camera hooks
        self._set_first_person_camera = set_first_person_camera
        self._reset_camera_tracking   = reset_camera_tracking
        self._is_first_person_camera  = is_first_person_camera
        self._fp_camera_enabled       = False

        # edge detection for axes
        self._prev_axis_sign = {
            "lx": 0,
            "ly": 0,
            "rx": 0,
            "ry": 0,
        }

    # ------------------------------------------------------------------
    # public interface
    # ------------------------------------------------------------------

    def start(self):
        """Start the background polling thread."""
        self._stop_ev.clear()
        self._thread = threading.Thread(target=self._run, daemon=True, name="ps-poll")
        self._thread.start()

    def stop(self):
        """Signal the polling thread to stop and wait for it."""
        self._stop_ev.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def is_connected(self):
        """Return True if a joystick is currently active."""
        return self._joystick is not None

    def force_disconnect(self):
        """
        Called from the main thread when a JOYDEVICEREMOVED event is received.
        Clears the joystick handle so the HUD immediately shows 'not connected'
        and the poll loop re-scans on its next iteration.
        """
        self._joystick = None
        self._airborne = False
        self._pending_num = None
        print("[PS] Joystick removed (forced disconnect).")

    def controller_name(self):
        """Return the SDL name string of the connected joystick, or empty string."""
        if self._joystick is not None:
            try:
                return self._joystick.get_name()
            except Exception:
                pass
        return ""

    # ------------------------------------------------------------------
    # internal helpers
    # ------------------------------------------------------------------

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
        if callable(self._is_first_person_camera):
            try:
                return bool(self._is_first_person_camera())
            except Exception:
                pass
        return self._fp_camera_enabled

    def _cmd_connect_selected(self):
        targets = self._targets()
        if not targets:
            self.last_cmd = ("Connect: no selected drone", time.monotonic())
            print("[PS] Connect skipped: no selected drone.")
            return

        for e in targets:
            try:
                e.start_connection()
            except Exception as exc:
                print(f"[PS] Connect failed for CF {e.num}: {exc}")

        self._airborne = False
        if self.controlled_num == 0:
            self.last_cmd = ("Connect selected drones: all CF", time.monotonic())
        else:
            self.last_cmd = (f"Connect selected drones: CF {self.controlled_num}", time.monotonic())

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
        On Linux/SDL2 triggers often rest at -1; on Windows they may rest at 0.
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
        if self._circle_held:
            return STEP_LARGE_M, STEP_LARGE_DEG
        return STEP_NORMAL_M, STEP_NORMAL_DEG

    def _move_duration(self):
        """Return movement duration, shortened when L2 is held."""
        return TURBO_MOVE_DURATION if self._trigger(self.AXIS_L2) > 0.5 else DEFAULT_MOVE_DURATION

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

    def _cmd_takeoff(self, height=0.5):
        for e in self._targets():
            t = self._thread_for(e)
            if t:
                t.takeoff(height)
        self._airborne = True

    def _cmd_land(self):
        for e in self._targets():
            t = self._thread_for(e)
            if t:
                t.land()
        self._airborne = False

    def _cmd_go_to(self, dx=0., dy=0., dz=0., dyaw_deg=0., duration=2.0):
        for e in self._targets():
            t = self._thread_for(e)
            if not t:
                continue

            # Resolve body-frame movement into world frame using current yaw.
            _, _, _, _, yaw, _, _ = e.state.get()
            yr = math.radians(yaw)

            # Body +x = forward, body +y = left (ENU convention)
            fwd = (math.cos(yr), math.sin(yr))
            lft = (-math.sin(yr), math.cos(yr))

            wx = fwd[0] * dx + lft[0] * dy
            wy = fwd[1] * dx + lft[1] * dy

            t.go_to(dx=float(wx), dy=float(wy), dz=dz, dyaw_deg=dyaw_deg, duration=duration)

    def _cmd_blink(self):
        for e in self._targets():
            t = self._thread_for(e)
            if t:
                t.blink_led()

    def _axis_rising(self, name, value):
        """
        Return True once when an axis crosses from centered to a direction.
        Used for one-shot discrete movement commands.
        """
        sign = 0
        if value > DEADZONE:
            sign = 1
        elif value < -DEADZONE:
            sign = -1

        prev = self._prev_axis_sign.get(name, 0)
        self._prev_axis_sign[name] = sign
        return sign != 0 and sign != prev

    # ------------------------------------------------------------------
    # polling loop
    # ------------------------------------------------------------------

    def _try_init_joystick(self):
        """Attempt to initialise the first available joystick."""
        if not PYGAME_AVAILABLE:
            return

        try:
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

        _prev_btn = {}

        def rising(idx):
            cur = self._btn(idx)
            prev = _prev_btn.get(idx, False)
            _prev_btn[idx] = cur
            return cur and not prev

        _prev_hat = (0, 0)

        while not self._stop_ev.is_set():
            t0 = time.monotonic()

            if self._joystick is None:
                self._try_init_joystick()
                if self._joystick is None:
                    time.sleep(2.0)
                    continue

            try:
                _ = self._joystick.get_name()
            except Exception:
                print("[PS] Joystick disconnected.")
                self._joystick = None
                self._airborne = False
                self._pending_num = None
                continue

            # Read raw inputs.
            lx  = self._axis(self.AXIS_LX)
            ly  = -self._axis(self.AXIS_LY)  # up = +1
            rx  = self._axis(self.AXIS_RX)
            ry  = -self._axis(self.AXIS_RY)
            l2  = self._trigger(self.AXIS_L2)
            r2  = self._trigger(self.AXIS_R2)
            hat = self._hat()

            # Step modifier buttons (hold)
            self._cross_held  = self._btn(self.BTN_CROSS)
            self._circle_held = self._btn(self.BTN_CIRCLE)
            step_m, step_deg  = self._step()
            move_duration = self._move_duration()

            # R2 blink LED
            if r2 > 0.5 and _prev_btn.get("r2_half", False) is False:
                _prev_btn["r2_half"] = True
                self._cmd_blink()
                self.last_cmd = ("Blink LED", time.monotonic())
                print("[PS] Blink LED!")
            elif r2 <= 0.5:
                _prev_btn["r2_half"] = False

            _ = l2  # used via _move_duration()

            if self.panel_open:
                elapsed = time.monotonic() - t0
                time.sleep(max(0.0, (1.0 / POLL_HZ) - elapsed))
                continue

            # Drone-selection workflow (L1 / R1 cycle, Share confirm)
            if rising(self.BTN_L1):
                if self._pending_num is None:
                    self._pending_num = self.controlled_num
                nums = sorted([0] + list(self._drones.keys()))
                idx = nums.index(self._pending_num) if self._pending_num in nums else 0
                self._pending_num = nums[(idx - 1) % len(nums)]
                if self._pending_num == 0:
                    self.last_cmd = ("Selecting all CF (press Share to confirm)", time.monotonic())
                    print("[PS] Selecting all CF (press Share to confirm)")
                else:
                    self.last_cmd = (f"Selecting CF {self._pending_num} (press Share to confirm)", time.monotonic())
                    print(f"[PS] Selecting CF {self._pending_num} (press Share to confirm)")

            if rising(self.BTN_R1):
                if self._pending_num is None:
                    self._pending_num = self.controlled_num
                nums = sorted([0] + list(self._drones.keys()))
                idx = nums.index(self._pending_num) if self._pending_num in nums else 0
                self._pending_num = nums[(idx + 1) % len(nums)]
                if self._pending_num == 0:
                    self.last_cmd = ("Selecting all CF (press Share to confirm)", time.monotonic())
                    print("[PS] Selecting all CF (press Share to confirm)")
                else:
                    self.last_cmd = (f"Selecting CF {self._pending_num} (press Share to confirm)", time.monotonic())
                    print(f"[PS] Selecting CF {self._pending_num} (press Share to confirm)")

            if rising(self.BTN_SHARE):
                if self._pending_num is not None:
                    self.controlled_num = self._pending_num
                    self._pending_num = None

                    if self.controlled_num == 0:
                        self.last_cmd = ("Now controlling all CF", time.monotonic())
                        print("[PS] Now controlling all CF")
                    else:
                        self.last_cmd = (f"Now controlling CF {self.controlled_num}", time.monotonic())
                        print(f"[PS] Now controlling CF {self.controlled_num}")

                    self._cmd_blink()

            # Cancel pending selection if any other significant input is given.
            if self._pending_num is not None:
                any_sig = (
                    abs(lx) > DEADZONE or abs(ly) > DEADZONE or
                    abs(rx) > DEADZONE or abs(ry) > DEADZONE or
                    hat != (0, 0) or self._btn(self.BTN_OPTIONS)
                )
                if any_sig:
                    self._pending_num = None
                    self.last_cmd = (f"Selection cancelled - staying on CF {self.controlled_num}", time.monotonic())
                    print(f"[PS] Selection cancelled - staying on CF {self.controlled_num}")

            # Connect selected drones.
            if rising(self.BTN_SQUARE):
                self._cmd_connect_selected()
                print("[PS] Connect selected drones")

            # Toggle first-person camera.
            if rising(self.BTN_TRIANGLE):
                if self._camera_is_first_person():
                    self._cmd_normal_camera()
                    print("[PS] Camera → normal view")
                else:
                    self._cmd_first_person_camera()
                    print("[PS] Camera → first-person")

            # Start flight: takeoff / land toggle.
            if rising(self.BTN_OPTIONS):
                if not self._airborne:
                    self._cmd_takeoff(STEP_LARGE_M)
                    self.last_cmd = (f"Takeoff → {STEP_NORMAL_M:.3f} m", time.monotonic())
                    print(f"[PS] Takeoff → {STEP_NORMAL_M:.3f} m")
                else:
                    self._cmd_land()
                    self.last_cmd = ("Land", time.monotonic())
                    print("[PS] Land")

            # D-pad: discrete vertical + yaw commands.
            hat_edge = hat != _prev_hat
            _prev_hat = hat
            if hat_edge and self._airborne:
                hx, hy = hat
                if hy == +1:
                    self._cmd_go_to(dz=+step_m, duration=move_duration)
                    self.last_cmd = (f"Up +{step_m:.3f} m", time.monotonic())
                elif hy == -1:
                    self._cmd_go_to(dz=-step_m, duration=move_duration)
                    self.last_cmd = (f"Down -{step_m:.3f} m", time.monotonic())
                elif hx == -1:
                    self._cmd_go_to(dyaw_deg=+step_deg, duration=move_duration)
                    self.last_cmd = (f"Turn Left +{step_deg:.1f}°", time.monotonic())
                elif hx == +1:
                    self._cmd_go_to(dyaw_deg=-step_deg, duration=move_duration)
                    self.last_cmd = (f"Turn Right -{step_deg:.1f}°", time.monotonic())

            # Sticks: forward/backward and slide left/right.
            # Both sticks are mapped the same way so either one can be used.
            if self._airborne:
                if self._axis_rising("ly", ly):
                    if ly > 0:
                        self._cmd_go_to(dx=+step_m, duration=move_duration)
                        self.last_cmd = (f"Forward +{step_m:.3f} m", time.monotonic())
                    else:
                        self._cmd_go_to(dx=-step_m, duration=move_duration)
                        self.last_cmd = (f"Backward -{step_m:.3f} m", time.monotonic())
                    print(f"[PS] {self.last_cmd[0]}")

                if self._axis_rising("lx", lx):
                    if lx > 0:
                        self._cmd_go_to(dy=-step_m, duration=move_duration)
                        self.last_cmd = (f"Slide Right +{step_m:.3f} m", time.monotonic())
                    else:
                        self._cmd_go_to(dy=+step_m, duration=move_duration)
                        self.last_cmd = (f"Slide Left -{step_m:.3f} m", time.monotonic())
                    print(f"[PS] {self.last_cmd[0]}")

                if self._axis_rising("ry", ry):
                    if ry > 0:
                        self._cmd_go_to(dx=+step_m, duration=move_duration)
                        self.last_cmd = (f"Forward +{step_m:.3f} m", time.monotonic())
                    else:
                        self._cmd_go_to(dx=-step_m, duration=move_duration)
                        self.last_cmd = (f"Backward -{step_m:.3f} m", time.monotonic())
                    print(f"[PS] {self.last_cmd[0]}")

                if self._axis_rising("rx", rx):
                    if rx > 0:
                        self._cmd_go_to(dy=-step_m, duration=move_duration)
                        self.last_cmd = (f"Slide Right +{step_m:.3f} m", time.monotonic())
                    else:
                        self._cmd_go_to(dy=+step_m, duration=move_duration)
                        self.last_cmd = (f"Slide Left -{step_m:.3f} m", time.monotonic())
                    print(f"[PS] {self.last_cmd[0]}")

            elapsed = time.monotonic() - t0
            time.sleep(max(0.0, (1.0 / POLL_HZ) - elapsed))
