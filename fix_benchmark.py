with open("test/benchmark_se3.py", "r") as f:
    content = f.read()

import_str = "import threading\nimport lcm\nfrom quadrotor_msgs import lcmt_quadrotor_setpoint\n"
if "import threading" not in content:
    content = import_str + content

fix_str = """
    print("[System] Waiting for processes to initialize...")
    
    # --- FIX: Publish hover setpoints during initialization so it doesn't fly to 0,0,0 ---
    init_setpoint = lcmt_quadrotor_setpoint()
    init_setpoint.position = [0.0, 0.0, 1.0]
    init_setpoint.velocity = [0.0, 0.0, 0.0]
    init_setpoint.yaw = 0.0
    init_setpoint.yaw_rate = 0.0
    lc_init = lcm.LCM()
    
    def pub_init():
        while not is_running:
            lc_init.publish("UAV_QUADROTOR_SETPOINT", init_setpoint.encode())
            time.sleep(0.01)
            
    threading.Thread(target=pub_init, daemon=True).start()
    # ---------------------------------------------------------------------------------

    time.sleep(3.0)
    print("[System] Both simulator and controller are ready! Starting benchmark...")
"""

content = content.replace('    print("[System] Waiting for processes to initialize...")\n    time.sleep(3.0)\n    print("[System] Both simulator and controller are ready! Starting benchmark...")', fix_str)

with open("test/benchmark_se3.py", "w") as f:
    f.write(content)
