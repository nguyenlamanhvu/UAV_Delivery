import lcm
from uav_delivery import lcmt_quadrotor_state, lcmt_quadrotor_command, lcmt_quadrotor_setpoint
import time

lc = lcm.LCM()

def on_state(channel, data):
    msg = lcmt_quadrotor_state.decode(data)
    print(f"STATE z={msg.position[2]:.2f} vz={msg.velocity[2]:.2f}")

def on_setpoint(channel, data):
    msg = lcmt_quadrotor_setpoint.decode(data)
    print(f"SETPOINT vz={msg.velocity[2]:.2f} mode={msg.mode}")

def on_command(channel, data):
    pass

lc.subscribe("UAV_QUADROTOR_STATE", on_state)
lc.subscribe("UAV_QUADROTOR_SETPOINT", on_setpoint)

print("Listening for LCM...")
try:
    while True:
        lc.handle()
except KeyboardInterrupt:
    pass
