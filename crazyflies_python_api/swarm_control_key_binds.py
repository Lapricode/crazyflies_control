import logging
import time
import threading
from pynput import keyboard
from cflib.crtp import init_drivers
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.crazyflie.high_level_commander import HighLevelCommander
from cflib.crazyflie.log import LogConfig


uris_nums = {
    # "radio://0/20/2M/E7E7E7E701": 1, \
    "radio://0/20/2M/E7E7E7E702": 2, \
    # "radio://0/20/2M/E7E7E7E703": 3
}
uris = list(uris_nums.keys())
crazyflies = {}
deck_attached_events = {uri: threading.Event() for uri in uris}
crazyflie_controlled = 2
pressed_keys = set()
def on_press(key):
    global crazyflie_controlled
    try:
        if key.char:
            if key.char.isdigit():
                crazyflie_controlled = int(key.char)
            else:
                pressed_keys.add(str(crazyflie_controlled) + key.char)
    except AttributeError:
        pass
def on_release(key):
    try:
        if str(crazyflie_controlled) + key.char in pressed_keys:
            pressed_keys.remove(str(crazyflie_controlled) + key.char)
    except AttributeError:
        pass
listener = keyboard.Listener(on_press = on_press, on_release = on_release)
listener.start()

def limit_vel(vel, vel_min, vel_max):
    return min(vel_max, max(vel_min, vel))

def leds_check(scf):
    scf.cf.param.set_value("led.bitmask", 255)
    time.sleep(2)
    scf.cf.param.set_value("led.bitmask", 0)
    time.sleep(2)

# def param_deck_lighthouse(uri):
#     print("here")
#     def callback(name, value_str):
#         value = int(value_str)
#         print(value)
#         if value:
#             print(f"[{uri}] Lighthouse deck attached.")
#             deck_attached_events[uri].set()
#         else:
#             print(f"[{uri}] Lighthouse deck NOT attached.")
#     return callback

def log_state_callback(uri):
    def callback(timestamp, data, logconf):
        print(f"[t = {timestamp}]: [{uri}] ->", \
                f"\n\t-> x = {data['stateEstimate.x']:.3f}, y = {data['stateEstimate.y']:.3f}, z = {data['stateEstimate.z']:.3f}", \
                f"\n\t-> roll = {data['stateEstimate.roll']:.3f}, pitch = {data['stateEstimate.pitch']:.3f}, yaw = {data['stateEstimate.yaw']:.3f}")
    return callback

def init_logging(scf, uri):
    lhdeck_value = scf.cf.param.get_value(complete_name = "deck.bcLighthouse4", timeout = 1)
    if int(lhdeck_value):
        print(f"[{uri}] -> Lighthouse deck attached!")
        deck_attached_events[uri].set()
    else:
        print(f"[{uri}] -> Lighthouse deck NOT attached!")
        return
    # scf.cf.param.add(group = "deck", name = "bcLighthouse4", cb = param_deck_lighthouse(uri))
    # if not deck_attached_events[uri].wait(timeout = 5):
    #     print(f"[{uri}] ERROR: Lighthouse deck not found. Exiting.")
    #     return

    log_state = LogConfig(name = "state", period_in_ms = 100)
    log_state.add_variable("stateEstimate.x", "float")
    log_state.add_variable("stateEstimate.y", "float")
    log_state.add_variable("stateEstimate.z", "float")
    log_state.add_variable("stateEstimate.roll", "float")
    log_state.add_variable("stateEstimate.pitch", "float")
    log_state.add_variable("stateEstimate.yaw", "float")

    scf.cf.log.add_config(log_state)
    log_state.data_received_cb.add_callback(log_state_callback(uri))
    log_state.start()
    while True:
        time.sleep(1)
    log_state.stop()

def fly(scf, uri):
    h = 0.5
    # with PositionHlCommander(scf, default_height = h) as pc:
    # with HighLevelCommander(scf) as hc:
    with MotionCommander(scf, default_height = h) as mc:
        print(f"[{uri}] -> Ready for command input ...")
        time.sleep(5)
        linear_vel = 0.1  # in meters/sec
        angular_vel = 10  # in degrees/second
        flying = True
        while True:
            time.sleep(0.1)
            if f"{uris_nums[uri]}w" in pressed_keys and flying:
                # print(f"[{uri}] -> Forward")
                mc.start_forward(linear_vel)
            elif f"{uris_nums[uri]}s" in pressed_keys and flying:
                # print(f"[{uri}] -> Back")
                mc.start_back(linear_vel)
            elif f"{uris_nums[uri]}a" in pressed_keys and flying:
                # print(f"[{uri}] -> Left")
                mc.start_left(linear_vel)
            elif f"{uris_nums[uri]}d" in pressed_keys and flying:
                # print(f"[{uri}] -> Right")
                mc.start_right(linear_vel)
            elif f"{uris_nums[uri]}i" in pressed_keys and flying:
                # print(f"[{uri}] -> Up")
                mc.start_up(linear_vel)
            elif f"{uris_nums[uri]}k" in pressed_keys and flying:
                # print(f"[{uri}] -> Down")
                mc.start_down(linear_vel)
            elif f"{uris_nums[uri]}j" in pressed_keys and flying:
                # print(f"[{uri}] -> Turn Left")
                mc.start_turn_left(angular_vel)
            elif f"{uris_nums[uri]}l" in pressed_keys and flying:
                # print(f"[{uri}] -> Turn Right")
                mc.start_turn_right(angular_vel)
            elif f"{uris_nums[uri]}z" in pressed_keys and not flying:
                # print(f"[{uri}] -> Take Off")
                mc.take_off(0.5, 0.2)
                flying = True
            elif f"{uris_nums[uri]}x" in pressed_keys and flying:
                # print(f"[{uri}] -> Land")
                mc.land(0.2)
                flying = False
            elif f"{uris_nums[uri]}r" in pressed_keys and flying:
                pass
                # hc.go_to(0.0, 0.0, h, linear_vel)
            elif f"{uris_nums[uri]}v" in pressed_keys and flying:
                linear_vel += 0.05
                angular_vel += 5
                linear_vel = limit_vel(linear_vel, 0.1, 1.0)
                angular_vel = limit_vel(angular_vel, 10, 90)
            elif f"{uris_nums[uri]}c" in pressed_keys and flying:
                linear_vel -= 0.05
                angular_vel -= 5
                linear_vel = limit_vel(linear_vel, 0.1, 1.0)
                angular_vel = limit_vel(angular_vel, 10, 90)
            elif f"{uris_nums[uri]}q" in pressed_keys and flying:
                # print(f"[{uri}] -> Quit flying and Land.")
                break
            elif flying:
                # print(f"[{uri}] -> Stop and hover.")
                mc.stop()
            else:
                pass

def move_linear_simple(scf, name = ""):
    with MotionCommander(scf, default_height = 0.5) as mc:
        print(f"[{name}] -> Taking off and moving ...")
        # time.sleep(1)
        # mc.forward(0.5)
        # time.sleep(1)
        # mc.back(0.5)
        # time.sleep(1)
        time.sleep(5)
        mc.forward(1.0)
        time.sleep(1)
        mc.turn_left(180)
        time.sleep(1)
        mc.forward(1.0)
        time.sleep(1)
        mc.turn_right(180)
        time.sleep(3)
        print(f"[{name}] -> Stopped moving.")

def init_cf(uri):
    scf = SyncCrazyflie(uri, cf = Crazyflie(rw_cache = "./cache"))
    scf.open_link()
    crazyflies[uri] = scf
    leds_check(scf)
    print(f"[{uri}] -> Connected!")
    return scf

def run_cf(uri):
    scf = init_cf(uri)
    threading.Thread(target = init_logging, args = (scf, uri), daemon = True).start()
    fly(scf, uri)
    print(f"[{uri}] -> Landing.")
    scf.close_link()


if __name__ == "__main__":
    logging.basicConfig(level = logging.ERROR)
    init_drivers()
    threads = []

    for uri in uris:
        t = threading.Thread(target = run_cf, args = (uri,))
        threads.append(t)
        t.start()
    for t in threads:
        t.join()

    # with SyncCrazyflie(uris[0], cf = Crazyflie(rw_cache = "./cache")) as scf:
    #     scf.cf.param.add_update_callback(group = "deck", name = "bcLighthouse4", cb = param_deck_lighthouse)
        
    #     time.sleep(1)

    #     log_pos = LogConfig(name = "position", period_in_ms = 10)
    #     log_pos.add_variable("stateEstimate.x", "float")
    #     log_pos.add_variable("stateEstimate.y", "float")
    #     log_pos.add_variable("stateEstimate.z", "float")
    #     scf.cf.log.add_config(log_pos)
    #     log_pos.data_received_cb.add_callback(log_pos_callback)
        
    #     if not deck_attached_events.wait(timeout = 5):
    #         print("NO Lighthouse deck detected!")
    #         sys.exit(1)
        
    #     scf.cf.platform.send_arming_request(True)
    #     time.sleep(1.0)
    #     log_pos.start()
    #     move_linear_simple(scf)
    #     time.sleep(600.0)
    #     log_pos.stop()
