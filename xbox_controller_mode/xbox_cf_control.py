import time
import pygame
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.utils import uri_helper


# ---------------- CONFIG ----------------
URI = uri_helper.uri_from_env(default='radio://0/90/2M/E7E7E7E7AB')

MAX_ROLL = 25       # degrees
MAX_PITCH = 25      # degrees
MAX_YAW = 200       # deg/s
MAX_THRUST = 65535  # max thrust
MIN_THRUST = 0  # idle thrust
DEADZONE = 0.1

# Buttons (Xbox typical mapping)
ARM_BUTTON = 0      # A button
KILL_BUTTON = 1     # B button


# ----------------------------------------

def apply_deadzone(val, dz):
    return 0 if abs(val) < dz else val

def scale_thrust(val):
    # val from [-1,1] → [0,1]
    val = (val + 1) / 2
    return int(MIN_THRUST + val * (MAX_THRUST - MIN_THRUST))

def main():
    print("Initializing Crazyflie...")
    cflib.crtp.init_drivers()

    cf = Crazyflie(rw_cache='./cache')

    print(f"Connecting to {URI}")
    cf.open_link(URI)

    time.sleep(2)  # allow connection

    # Init controller
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("No controller found!")
        return

    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"Controller connected: {js.get_name()}")

    armed = False

    try:
        while True:
            pygame.event.pump()

            # Read axes
            roll_input  = apply_deadzone(js.get_axis(0), DEADZONE)
            pitch_input = apply_deadzone(js.get_axis(1), DEADZONE)
            yaw_input   = apply_deadzone(js.get_axis(3), DEADZONE)
            thrust_input = -js.get_axis(4)  # invert stick

            # Scale
            roll  = roll_input * MAX_ROLL
            pitch = -pitch_input * MAX_PITCH
            yaw   = yaw_input * MAX_YAW
            thrust = scale_thrust(thrust_input)

            # Buttons
            arm_pressed = js.get_button(ARM_BUTTON)
            kill_pressed = js.get_button(KILL_BUTTON)

            # ARM
            if arm_pressed:
                if not armed:
                    print("ARMED")
                armed = True

            # KILL SWITCH
            if kill_pressed:
                print("EMERGENCY STOP")
                cf.commander.send_stop_setpoint()
                armed = False
                continue

            # SEND COMMANDS
            if armed:
                cf.commander.send_setpoint(roll, pitch, yaw, thrust)
            else:
                cf.commander.send_setpoint(0, 0, 0, 0)

            # Debug print
            print(f"R:{roll:.1f} P:{pitch:.1f} Y:{yaw:.1f} T:{thrust}", end="\r")

            time.sleep(0.01)  # 100 Hz

    except KeyboardInterrupt:
        print("\nExiting...")

    finally:
        cf.commander.send_stop_setpoint()
        cf.close_link()


if __name__ == "__main__":
    main()
