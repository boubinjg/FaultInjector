import sys, struct, time, os 
from curses import ascii
from pymavlink import mavparm
from pymavlink import mavutil
import socket
from pymavlink.dialects.v10 import common as mavlink

m = mavutil.mavlink_connection('tcp:127.0.0.1:5763', dialect='ardupilotmega', write=True, input=False)

print("Waiting for APM heartbeat")
msg = m.recv_match(type='HEARTBEAT', blocking=True)
print("Heartbeat from APM (system %u, component %u)" % (m.target_system, m.target_system))

mav_param = mavparm.MAVParmDict()
mav_param.mavset(m, "SIM_WIND_DIR", float(70))
mav_param.mavset(m, "SIM_WIND_SPD", float(1))
#m.parm_set_send("DIM_WIND_DIR", float(70))
