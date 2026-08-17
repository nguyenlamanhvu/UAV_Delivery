import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import lcm
import time
import subprocess
import threading
import sys
import math

from uav_delivery import lcmt_quadrotor_state, lcmt_quadrotor_setpoint

actual_p = []
actual_t = []
des_p = []
des_t = []

def state_handler(channel, data):
    msg = lcmt_quadrotor_state.decode(data)
    actual_t.append(msg.utime / 1e6)
    actual_p.append([msg.position[0], msg.position[1], msg.position[2]])

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=0")
lc.subscribe("UAV_QUADROTOR_STATE", state_handler)

def lcm_listen():
    while True:
        lc.handle()

t = threading.Thread(target=lcm_listen, daemon=True)
t.start()

print("Launching sim...")
sim = subprocess.Popen(["./bazel-bin/quadrotor_sim", "--no_console_log"])
ctrl = subprocess.Popen(["./bazel-bin/quadrotor_se3_controller"])
time.sleep(1.0)

print("Sending step input...")
start_time = time.time()
while time.time() - start_time < 5.0:
    msg = lcmt_quadrotor_setpoint()
    msg.utime = int((time.time() - start_time) * 1e6)
    msg.mode = 0
    msg.position = [5.0, 0.0, 1.0] # 5m step in X
    msg.velocity = [0.0, 0.0, 0.0]
    msg.acceleration = [0.0, 0.0, 0.0]
    msg.yaw = 0.0
    msg.yaw_rate = 0.0
    lc.publish("UAV_QUADROTOR_SETPOINT", msg.encode())
    
    des_t.append(msg.utime / 1e6)
    des_p.append([5.0, 0.0, 1.0])
    time.sleep(0.01)

sim.kill()
ctrl.kill()

actual_p = np.array(actual_p)
actual_t = np.array(actual_t)

plt.figure(figsize=(10, 5))
plt.plot(actual_t, actual_p[:, 0], label='Actual X')
plt.axhline(5.0, color='r', linestyle='--', label='Setpoint')
plt.title("Step Response in X")
plt.xlabel("Time (s)")
plt.ylabel("Position (m)")
plt.legend()
plt.grid(True)
plt.savefig("step_response.png")
print("Saved step_response.png")

