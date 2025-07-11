import time
import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie


uris = {
    "radio://0/20/2M/E7E7E7E701", \
    "radio://0/20/2M/E7E7E7E702", \
    "radio://0/20/2M/E7E7E7E703", \
    "radio://0/20/2M/E7E7E7E704"
}

def activate_led_bit_mask(scf):
    scf.cf.param.set_value("led.bitmask", 255)

def deactivate_led_bit_mask(scf):
    scf.cf.param.set_value("led.bitmask", 0)

def light_check(scf):
    activate_led_bit_mask(scf)
    time.sleep(2)
    deactivate_led_bit_mask(scf)

def take_off(scf):
    commander = scf.cf.high_level_commander
    commander.takeoff(1.0, 2.0)
    time.sleep(3)

def land(scf):
    commander = scf.cf.high_level_commander
    commander.land(0.0, 2.0)
    time.sleep(2)
    commander.stop()

def hover_sequence(scf):
    take_off(scf)
    land(scf)

def run_square_sequence(scf):
    box_size = 1
    flight_time = 2
    commander = scf.cf.high_level_commander
    commander.go_to(box_size, 0, 0, 0, flight_time, relative = True)
    time.sleep(flight_time)
    commander.go_to(0, box_size, 0, 0, flight_time, relative = True)
    time.sleep(flight_time)
    commander.go_to(-box_size, 0, 0, 0, flight_time, trelative = True)
    time.sleep(flight_time)
    commander.go_to(0, -box_size, 0, 0, flight_time, trelative = True)
    time.sleep(flight_time)

h = 0.0  # remain at constant height similar to take off height
x0, y0 = +1.0, +1.0
x1, y1 = -1.0, -1.0

#    x   y   z  time
sequence0 = [
    (x1, y0, h, 3.0),
    (x0, y1, h, 3.0),
    (x0,  0, h, 3.0),
]

sequence1 = [
    (x0, y0, h, 3.0),
    (x1, y1, h, 3.0),
    (.0, y1, h, 3.0),
]

sequence2 = [
    (x0, y1, h, 3.0),
    (x1, y0, h, 3.0),
    (x1,  0, h, 3.0),
]

sequence3 = [
    (x1, y1, h, 3.0),
    (x0, y0, h, 3.0),
    (.0, y0, h, 3.0),
]

seq_args = {
    uris[0]: [sequence0],
    uris[1]: [sequence1],
    uris[2]: [sequence2],
    uris[3]: [sequence3],
}

def run_sequence(scf: SyncCrazyflie, sequence):
    for arguments in sequence:
        commander = scf.cf.high_level_commander
        x, y, z = arguments[0], arguments[1], arguments[2]
        duration = arguments[3]
        print("Setting position {} to cf {}".format((x, y, z), scf.cf.link_uri))
        commander.go_to(x, y, z, 0, duration, relative = True)
        time.sleep(duration)


if __name__ == "__main__":
    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache = "./cache")
    with Swarm(uris, factory = factory) as swarm:
        print("Connected to Crazyflies")
        swarm.parallel_safe(light_check)
        swarm.reset_estimators()
        # swarm.sequential(hover_sequence)
        swarm.parallel_safe(take_off)
        swarm.parallel_safe(run_square_sequence)
        swarm.parallel_safe(run_sequence, args_dict = seq_args)
        swarm.parallel_safe(land)
