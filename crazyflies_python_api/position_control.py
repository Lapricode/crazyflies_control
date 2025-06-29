import logging
import time
from cflib.crtp import init_drivers
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
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
        print(f" - [t = {timestamp}]: [{uri}] -> {logconf.name}", \
                f"\n\t-> Battery level = {data['pm.batteryLevel']:3d} %")
    return callback

def log_motors_callback(uri):
    def callback(timestamp, data, logconf):
        print(f" - [t = {timestamp}]: [{uri}] -> {logconf.name}", \
                f"\n\t-> m1 = {data['motor.m1']}, m2 = {data['motor.m2']}, m3 = {data['motor.m3']}, m4 = {data['motor.m4']}")
    return callback


if __name__ == "__main__":
    logging.basicConfig(level = logging.ERROR)
    init_drivers()

    uri = make_uri(0, 20, "2M", "E7E7E7E702")
    scf = SyncCrazyflie(uri, cf = Crazyflie(rw_cache = "./cache"))
    scf.open_link()
    print(f"Successfully connected to {uri}!")
    check_connection_leds_signal(scf)

    log_battery = LogConfig(name = "Battery", period_in_ms = 1000)
    log_battery.add_variable("pm.batteryLevel", "uint8_t")
    scf.cf.log.add_config(log_battery)
    log_battery.data_received_cb.add_callback(log_battery_callback(uri))
    
    scf.cf.param.set_value("motorPowerSet.enable", 1)
    scf.cf.param.set_value("motorPowerSet.m1", 0)
    scf.cf.param.set_value("motorPowerSet.m2", 0)
    scf.cf.param.set_value("motorPowerSet.m3", 0)
    scf.cf.param.set_value("motorPowerSet.m4", 0)

    log_motors = LogConfig(name = "Motors speed", period_in_ms = 1000)
    log_motors.add_variable("motor.m1", "uint16_t")
    log_motors.add_variable("motor.m2", "uint16_t")
    log_motors.add_variable("motor.m3", "uint16_t")
    log_motors.add_variable("motor.m4", "uint16_t")
    scf.cf.log.add_config(log_motors)
    log_motors.data_received_cb.add_callback(log_motors_callback(uri))

    log_battery.start()
    time.sleep(2)
    log_battery.stop()
    log_motors.start()
    time.sleep(2)
    log_motors.stop()

    scf.close_link()
