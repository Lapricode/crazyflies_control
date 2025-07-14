import time
import numpy as np
import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie


uris = [
    "radio://0/20/2M/E7E7E7E701", \
    "radio://0/20/2M/E7E7E7E702", \
    "radio://0/20/2M/E7E7E7E703", \
    "radio://0/20/2M/E7E7E7E704"
]

def light_check(scf):
    scf.cf.param.set_value("led.bitmask", 255)
    time.sleep(2)
    scf.cf.param.set_value("led.bitmask", 0)

def take_off(scf, height, flight_time):
    commander = scf.cf.high_level_commander
    commander.takeoff(height, flight_time)
    time.sleep(flight_time)

def land(scf, height, flight_time):
    commander = scf.cf.high_level_commander
    commander.land(height, flight_time)
    time.sleep(flight_time)
    commander.stop()

def hover_sequence(scf, height, flight_time):
    take_off(scf, height, flight_time)
    land(scf, 0.0, flight_time)

def run_square_sequence(scf):
    box_size = 1
    flight_time = 2
    commander = scf.cf.high_level_commander
    commander.go_to(box_size, 0.0, 0.0, 0.0, flight_time, relative = True)
    time.sleep(flight_time)
    commander.go_to(0.0, box_size, 0.0, 0.0, flight_time, relative = True)
    time.sleep(flight_time)
    commander.go_to(-box_size, 0.0, 0.0, 0.0, flight_time, relative = True)
    time.sleep(flight_time)
    commander.go_to(0.0, -box_size, 0.0, 0.0, flight_time, relative = True)
    time.sleep(flight_time)

def run_circle_sequence(scf):
    flight_time = 20.0  # seconds
    dt = 0.2  # time step in seconds
    steps = int(flight_time / dt)  # movement steps
    r = 1.5  # m
    h = 1.0  # m
    circle_freq = 1 / flight_time  # Hz
    up_down_dist = 0.2  # m
    up_down_freq = circle_freq / 2.0  # Hz
    commander = scf.cf.high_level_commander
    cf_num = list(uris).index(scf.cf.link_uri)
    cfs_N = len(uris)
    phi0 = cf_num * (2.0*np.pi / cfs_N)
    reach_init_pos_time = 5.0
    commander.go_to(r * np.cos(phi0), r * np.sin(phi0), h, 0.0, reach_init_pos_time, relative = False)
    time.sleep(reach_init_pos_time)
    for k in range(steps):
        phi_circle = 2.0*np.pi*circle_freq * k*dt + phi0
        phi_up_down = 2.0*np.pi*up_down_freq * k*dt
        commander.go_to(r * np.cos(phi_circle), r * np.sin(phi_circle), h + up_down_dist * np.sin(phi_up_down), 0.0, dt, relative = False)
        time.sleep(dt)

def run_sequence(scf: SyncCrazyflie, sequence):
    h = 0.0
    x0, y0 = +1.0, +1.0
    x1, y1 = -1.0, -1.0
    sequence0 = [(x1, y0, h, 3.0), (x0, y1, h, 3.0), (x0, 0.0, h, 3.0)]
    sequence1 = [(x0, y0, h, 3.0), (x1, y1, h, 3.0), (0.0, y1, h, 3.0)]
    sequence2 = [(x0, y1, h, 3.0), (x1, y0, h, 3.0), (x1, 0.0, h, 3.0)]
    sequence3 = [(x1, y1, h, 3.0), (x0, y0, h, 3.0), (0.0, y0, h, 3.0)]
    sequence_total = {
        uris[0]: [sequence0],
        uris[1]: [sequence1],
        uris[2]: [sequence2],
        uris[3]: [sequence3],
    }
    for uri, sequence in sequence_total.items():
        for x, y, z, duration in sequence:
            commander = scf.cf.high_level_commander
            # print("Setting position {} to cf {}".format((x, y, z), scf.cf.link_uri))
            commander.go_to(x, y, z, 0, duration, relative=True)
            time.sleep(duration)


if __name__ == "__main__":
    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache = "./cache")
    with Swarm(uris, factory = factory) as swarm:
        print("Connected to Crazyflies")
        swarm.parallel_safe(light_check)
        swarm.reset_estimators()
        swarm.sequential(lambda scf: hover_sequence(scf, 0.5, 2.0))
        swarm.parallel_safe(lambda scf: take_off(scf, 0.5, 5.0))
        swarm.parallel_safe(run_circle_sequence)
        # swarm.parallel_safe(run_square_sequence)
        # swarm.parallel_safe(run_sequence, args_dict = seq_args)
        swarm.parallel_safe(lambda scf: land(scf, 0.0, 5.0))
