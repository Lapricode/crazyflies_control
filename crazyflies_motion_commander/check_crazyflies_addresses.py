import logging
import time
import sys
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig


uris = [
    "radio://0/20/2M/E7E7E7E701", \
    "radio://0/20/2M/E7E7E7E702", \
    "radio://0/20/2M/E7E7E7E703", \
    "radio://0/20/2M/E7E7E7E704", \
    "radio://0/20/2M/E7E7E7E705"
]

def leds_check(scf):
    try:
        scf.cf.param.set_value("led.bitmask", 255)
        time.sleep(2)
        scf.cf.param.set_value("led.bitmask", 0)
        time.sleep(2)
    except Exception as e:
        print(f"Error during LED check: {e}")


if __name__ == "__main__":
    logging.basicConfig(level = logging.ERROR)
    cflib.crtp.init_drivers()

    working_uris = []
    failed_uris = []

    for uri in uris:
        try:
            print(f"Trying to connect to {uri} ...")
            with SyncCrazyflie(uri, cf = Crazyflie(rw_cache = "./rw_cache")) as scf:
                print(f"\tSuccessfully connected to {uri}")
                working_uris.append(uri)
                leds_check(scf)
        except Exception as e:
            print(f"\tFailed to connect to {uri}: {e}")
            failed_uris.append(uri)
    
    print("\n--- Crazyflies addresses check ---")
    print("Working URIs:")
    for uri in working_uris:
        print(f"  ✔ {uri}")
    print("Failed URIs:")
    for uri in failed_uris:
        print(f"  ✖ {uri}")