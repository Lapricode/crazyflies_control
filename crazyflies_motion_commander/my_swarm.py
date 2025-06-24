import logging
import time
import sys
from threading import Event
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.positioning.motion_commander import MotionCommander


uris = {
    "radio://0/20/2M/E7E7E7E701", \
    "radio://0/20/2M/E7E7E7E702", \
    "radio://0/20/2M/E7E7E7E703"
}
deck_attached_events = {uri: Event() for uri in uris}

def leds_check(scf):
    scf.cf.param.set_value("led.bitmask", 255)
    time.sleep(2)
    scf.cf.param.set_value("led.bitmask", 0)
    time.sleep(2)

def param_deck_lighthouse(uri):
    def callback(name, value_str):
        value = int(value_str)
        if value:
            print(f"[{uri}] Lighthouse deck attached.")
            deck_attached_events[uri].set()
        else:
            print(f"[{uri}] Lighthouse deck NOT attached.")
    return callback

def log_pos_callback(uri):
    def callback(timestamp, data, logconf):
        print(f"[{timestamp}] -> [{uri}]: x = {data['stateEstimate.x']:.3f}, "
              f"y = {data['stateEstimate.y']:.3f}, z = {data['stateEstimate.z']:.3f}")
    return callback

def init_logging(scf, uri):
    scf.cf.param.add_update_callback(group = "deck", name = "bcLighthouse4", cb = param_deck_lighthouse(uri))
    if not deck_attached_events[uri].wait(timeout = 5):
        print(f"[{uri}] ERROR: Lighthouse deck not found. Exiting.")
        return

    log_conf = LogConfig(name = "position", period_in_ms = 100)
    log_conf.add_variable("stateEstimate.x", "float")
    log_conf.add_variable("stateEstimate.y", "float")
    log_conf.add_variable("stateEstimate.z", "float")

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(log_pos_callback(uri))
    log_conf.start()

    # Keep log running for 600 seconds (10 min)
    time.sleep(600.0)
    log_conf.stop()


def move_linear_simple(scf):
    with MotionCommander(scf, default_height = 0.5) as mc:
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
    logging.basicConfig(level = logging.ERROR)
    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache = "./cache")
    with Swarm(uris, factory = factory) as swarm:
        swarm.parallel_safe(leds_check)
        # swarm.parallel_safe(init_logging)
        for i in range(len(uris)):
            move_linear_simple((swarm._cfs)[list(uris)[i]])

    # with SyncCrazyflie(list(uris)[0], cf = Crazyflie(rw_cache = "./cache")) as scf:
    #     scf.cf.param.add_update_callback(group = "deck", name = "bcLighthouse4", cb = param_deck_lighthouse)
        
    #     time.sleep(1)

    #     log_pos = LogConfig(name = "position", period_in_ms = 10)
    #     log_pos.add_variable("stateEstimate.x", "float")
    #     log_pos.add_variable("stateEstimate.y", "float")
    #     log_pos.add_variable("stateEstimate.z", "float")
    #     scf.cf.log.add_config(log_pos)
    #     log_pos.data_received_cb.add_callback(log_pos_callback)
        
    #     if not deck_attached_event.wait(timeout = 5):
    #         print("NO Lighthouse deck detected!")
    #         sys.exit(1)
        
    #     scf.cf.platform.send_arming_request(True)
    #     time.sleep(1.0)
    #     log_pos.start()
    #     move_linear_simple(scf)
    #     time.sleep(600.0)
    #     log_pos.stop()
