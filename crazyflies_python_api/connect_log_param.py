import logging
import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger


uri = uri_helper.uri_from_env(default = "radio://0/80/2M/E7E7E7E701")
logging.basicConfig(level = logging.ERROR)

def simple_connect():
    print("CONNECTION SUCCESS!")
    time.sleep(3)
    print("LOST CONNECTION!")

def param_stab_est_callback(name, value):
    print("The crazyflie has parameter " + name + " set as number: " + value)

def simple_param_async(scf, groupstr, namestr):
    cf = scf.cf
    full_name = groupstr + "." + namestr
    cf.param.add_update_callback(group = groupstr, name = namestr, cb = param_stab_est_callback)
    time.sleep(3)
    cf.param.set_value(full_name, 2)
    time.sleep(3)
    cf.param.set_value(full_name, 1)
    time.sleep(3)

def log_stab_callback(timestamp, data, logconf):
    print("[%d][%s]: roll = %f, pitch = %f, yaw = %f" % \
            (timestamp, logconf.name, data["stabilizer.roll"], data["stabilizer.pitch"], data["stabilizer.yaw"]))

def simple_log_async(scf, logconf):
    cf = scf.cf
    cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(log_stab_callback)
    logconf.start()
    time.sleep(5)
    logconf.stop()

def simple_log(scf, logconf):
    with SyncLogger(scf, lg_stab) as logger:
        for log_entry in logger:
            timestamp = log_entry[0]
            data = log_entry[1]
            logconf_name = log_entry[2]
            print("[%d][%s]: roll = %f, pitch = %f, yaw = %f" % \
                    (timestamp, logconf.name, data["stabilizer.roll"], data["stabilizer.pitch"], data["stabilizer.yaw"]))


if __name__ == "__main__":
    cflib.crtp.init_drivers()

    lg_stab = LogConfig(name = "stabilizer", period_in_ms = 10)
    lg_stab.add_variable("stabilizer.roll", "float")
    lg_stab.add_variable("stabilizer.pitch", "float")
    lg_stab.add_variable("stabilizer.yaw", "float")

    group = "stabilizer"
    name = "estimator"

    with SyncCrazyflie(link_uri = uri, cf = Crazyflie(rw_cache = "./cache")) as scf:
        # simple_connect()
        # simple_log(scf, lg_stab)
        # simple_log_async(scf, lg_stab)
        simple_param_async(scf, group, name)