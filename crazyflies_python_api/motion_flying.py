import logging
import sys
import time
from threading import Event
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper


uri = uri_helper.uri_from_env(default = "radio://0/20/2M/E7E7E7E701")
deck_attached_event = Event()

DEFAULT_HEIGHT = 0.5
BOX_LIMIT = 0.5
est_pos = [0, 0]

def log_pos_callback(timestamp, data, logconf):
    global est_pos
    print("[%d] -> [%s]: x = %f, y = %f, z = %f" % \
            (timestamp, logconf.name, data["stateEstimate.x"], data["stateEstimate.y"], data["stateEstimate.z"]))
    est_pos[0] = data["stateEstimate.x"]
    est_pos[1] = data["stateEstimate.y"]

def param_deck_flow(name, value_str):
    value = int(value_str)
    print(value)
    if value:
        deck_attached_event.set()
        print("Deck '%s' is attached!" % (name))
    else:
        print("Deck '%s' is NOT attached!" % (name))

def take_off_simple(scf):
    with MotionCommander(scf, default_height = DEFAULT_HEIGHT) as mc:
        time.sleep(3)
        mc.stop()

def move_box_limit(scf):
    with MotionCommander(scf, default_height = DEFAULT_HEIGHT) as mc:
        body_x_cmd = 0.2
        body_y_cmd = 0.1
        max_vel = 0.2
        while(1):
            if est_pos[0] > BOX_LIMIT:
                body_x_cmd = -max_vel
            elif est_pos[0] < -BOX_LIMIT:
                body_x_cmd = max_vel
            if est_pos[1] > BOX_LIMIT:
                body_y_cmd = -max_vel
            elif est_pos[1] < -BOX_LIMIT:
                body_y_cmd = max_vel
            mc.start_linear_motion(body_x_cmd, body_y_cmd, 0)
            time.sleep(0.1)
        # mc.start_forward()
        # while(1):
        #     if est_pos[0] > BOX_LIMIT:
        #         mc.start_back()
        #     elif est_pos[0] < BOX_LIMIT:
        #         mc.start_forward()            
        #     time.sleep(0.1)

def move_linear_simple(scf):
    with MotionCommander(scf, default_height = DEFAULT_HEIGHT) as mc:
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


if __name__ == "__main__":
    cflib.crtp.init_drivers()
    with SyncCrazyflie(uri, cf = Crazyflie(rw_cache = "./cache")) as scf:
        scf.cf.param.add_update_callback(group = "deck", name = "bcLighthouse4", cb = param_deck_flow)
        
        time.sleep(1)

        logconf = LogConfig(name = "position", period_in_ms = 10)
        logconf.add_variable("stateEstimate.x", "float")
        logconf.add_variable("stateEstimate.y", "float")
        logconf.add_variable("stateEstimate.z", "float")
        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_pos_callback)
        
        if not deck_attached_event.wait(timeout = 5):
            print("NO flowdeck detected!")
            sys.exit(1)
        
        scf.cf.platform.send_arming_request(True)
        time.sleep(1.0)
        logconf.start()
        # take_off_simple(scf)
        # move_linear_simple(scf)
        move_box_limit(scf)
        logconf.stop()
