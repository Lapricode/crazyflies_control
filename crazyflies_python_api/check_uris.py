import time
import logging
from cflib.crtp import init_drivers
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig


def make_uri(radio_id, channel, datarate, address_hex):
    return f"radio://{radio_id}/{channel}/{datarate}/{address_hex}"

def check_connection_leds_signal(scf):
    try:
        scf.cf.param.set_value("led.bitmask", 255)
        time.sleep(2)
        scf.cf.param.set_value("led.bitmask", 0)
        time.sleep(1)
    except Exception as e:
        print(f"\t-> Error during connection leds signal: {e}")

def log_battery_callback(uri):
    def callback(timestamp, data, logconf):
        print(f"\t - t = [{timestamp}]: [{uri}] -> {logconf.name}", \
                f"\n\t\t-> Battery level = {data['pm.batteryLevel']:3d} %")
    return callback

def detect_crazyflies_uris(tested_uris):
    logging.basicConfig(level = logging.ERROR)
    init_drivers()

    detected_uris = []
    failed_uris = []

    for uri in tested_uris:
        try:
            print(f"Trying to connect to {uri} ...")
            with SyncCrazyflie(uri, cf = Crazyflie(rw_cache = "./cache")) as scf:
                print(f"\t - Successfully connected to {uri}!")
                detected_uris.append(uri)
                check_connection_leds_signal(scf)
                log_battery = LogConfig(name = "Battery", period_in_ms = 1000)
                log_battery.add_variable("pm.batteryLevel", "uint8_t")
                scf.cf.log.add_config(log_battery)
                log_battery.data_received_cb.add_callback(log_battery_callback(uri))
                log_battery.start()
                time.sleep(2)
                log_battery.stop()
        except Exception as e:
            print(f"\t - Failed to connect to {uri}!: {e}")
            failed_uris.append(uri)
    
    print("\n--- Crazyflies addresses check ---")
    print("Detected URIs:")
    for uri in detected_uris:
        print(f"  ✔ {uri}")
    print("Failed URIs:")
    for uri in failed_uris:
        print(f"  ✖ {uri}")


uris = [
    make_uri(0, 10, "2M", "E7E7E7E701"), \
    make_uri(0, 40, "2M", "E7E7E7E704"), \
    make_uri(0, 80, "2M", "E7E7E7E708"), \
    make_uri(0, 90, "2M", "E7E7E7E709"), \
]
# uris = []
# for offset in range(0, 20):
#     address = f"E7E7E7E7{offset:02d}"
#     # uris.append(make_uri(0, 80, "2M", address))
#     for channel in range(0, 126, 5):
#         uris.append(make_uri(0, channel, "2M", address))

detect_crazyflies_uris(uris)
