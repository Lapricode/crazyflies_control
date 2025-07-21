import numpy as np
import logging
import time
import sys
from cflib.crtp import init_drivers
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.crazyflie.high_level_commander import HighLevelCommander
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
        print(f"Error during connection leds signal!: {e}")

def check_lighthouse_deck(scf):
    # scf.cf.param.add_update_callback(group = "deck", name = "bcLighthouse4", cb = param_lighthouse_deck)
    lhdeck_value = scf.cf.param.get_value(complete_name = "deck.bcLighthouse4", timeout = 1)
    if int(lhdeck_value):
        print(f"[{uri}] -> Lighthouse deck attached!")
    else:
        print(f"[{uri}] -> Lighthouse deck NOT attached!")
        sys.exit(1)

def log_state_callback(uri):
    def callback(timestamp, data, logconf):
        print(f"[t = {timestamp}]: [{uri}] ->", \
                f"\n\t-> x = {data['stateEstimate.x']:.3f}, y = {data['stateEstimate.y']:.3f}, z = {data['stateEstimate.z']:.3f}", \
                f"\n\t-> roll = {data['stateEstimate.roll']:.3f}, pitch = {data['stateEstimate.pitch']:.3f}, yaw = {data['stateEstimate.yaw']:.3f}")
    return callback

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

def create_target_position():
    xd, yd, zd, yawd = 0.0, 0.0, 0.0, 0.0
    fly_target = True
    while True:
        try:
            xd = float(input("x desired (m): ").strip())
            yd = float(input("y desired (m): ").strip())
            zd = float(input("z desired (m): ").strip())
            yawd = float(input("yaw desired (degrees): ").strip())
        except ValueError:
            print("❌ Invalid number entered. Try again:")
            continue
        confirm_target = input("Press ENTER to proceed with the target, or type 'q' and press ENTER to land: ")
        if confirm_target == "":
            print("✅ Confirmed, proceeding to fly!")
            break
        elif confirm_target == "q":
            print("⚠️ Landing now!")
            fly_target = False
            break
    return xd, yd, zd, yawd, fly_target


if __name__ == "__main__":
    logging.basicConfig(level = logging.ERROR)
    init_drivers()
    default_height = 0.5  # default height at takeoff in meters

    # set the uri of the crazyflie
    uri = make_uri(0, 100, "2M", "E7E7E7E710")
    
    # establish the connection with the crazyflie
    try:
        scf = SyncCrazyflie(uri, cf = Crazyflie(rw_cache = "./cache"))
        scf.open_link()
        print(f"[{uri}] -> Connected!")
        check_connection_leds_signal(scf)
        # check_lighthouse_deck(scf)
    except Exception as e:
        print(f"Failed to connect to {uri}!: {e}")
        sys.exit(1)

    # scf.cf.param.set_value("motorPowerSet.enable", 1)
    # scf.cf.param.set_value("motorPowerSet.m1", 0)
    # scf.cf.param.set_value("motorPowerSet.m2", 0)
    # scf.cf.param.set_value("motorPowerSet.m3", 0)
    # scf.cf.param.set_value("motorPowerSet.m4", 0)

    # set your preferred controller, for example: 1 for default PID and 6 for your OutOfTree controller
    scf.cf.param.set_value("stabilizer.controller", 6)
    # scf.cf.param.set_value("stabilizer.estimator", 3)

    time.sleep(5)
    print(scf.cf.param.get_value("stabilizer.controller", timeout = 5))
    print(scf.cf.param.get_value("stabilizer.estimator", timeout = 5))

    hlc = HighLevelCommander(scf.cf)
    time.sleep(1)

    log_battery = LogConfig(name = "Battery", period_in_ms = 1000)
    log_battery.add_variable("pm.batteryLevel", "uint8_t")
    scf.cf.log.add_config(log_battery)
    log_battery.data_received_cb.add_callback(log_battery_callback(uri))
    
    log_motors = LogConfig(name = "Motors speed", period_in_ms = 100)
    log_motors.add_variable("motor.m1", "uint16_t")
    log_motors.add_variable("motor.m2", "uint16_t")
    log_motors.add_variable("motor.m3", "uint16_t")
    log_motors.add_variable("motor.m4", "uint16_t")
    scf.cf.log.add_config(log_motors)
    log_motors.data_received_cb.add_callback(log_motors_callback(uri))

    log_state = LogConfig(name = "Current State", period_in_ms = 100)
    log_state.add_variable("stateEstimate.x", "float")
    log_state.add_variable("stateEstimate.y", "float")
    log_state.add_variable("stateEstimate.z", "float")
    log_state.add_variable("stateEstimate.roll", "float")
    log_state.add_variable("stateEstimate.pitch", "float")
    log_state.add_variable("stateEstimate.yaw", "float")
    scf.cf.log.add_config(log_state)
    log_state.data_received_cb.add_callback(log_state_callback(uri))

    # log_state_error = LogConfig(name = "State Error", period_in_ms = 100)

    scf.cf.console.receivedChar.add_callback(lambda text: print(text))

    log_state.start()
    # log_battery.start()
    # log_motors.start()

    takeoff_time = 2.0
    land_time = 3.0
    goto_time = 10.0
    try:
        hlc.takeoff(absolute_height_m = default_height, duration_s = takeoff_time)
        time.sleep(takeoff_time)
        while True:
            time.sleep(1)
            # xd, yd, zd, yawd, fly_target = create_target_position()
            xd, yd, zd, yawd, fly_target = 1.0, 1.0, 1.0, 0.0, True
            if fly_target:
                hlc.go_to(x = xd, y = yd, z = zd, yaw = np.deg2rad(yawd), duration_s = goto_time)
                time.sleep(goto_time)
            else:
                hlc.land(absolute_height_m = 0.0, duration_s = land_time)
                time.sleep(land_time)
                break
    finally:
        try:
            hlc.land(absolute_height_m = 0.0, duration_s = land_time)
            time.sleep(land_time)
        except:
            pass
        scf.close_link()

    # log_motors.stop()
    # log_battery.stop()
    log_state.stop()

    scf.close_link()

# check "ctrltarget" log variables
