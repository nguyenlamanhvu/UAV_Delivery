import http.server
import socketserver
import json
import lcm
import time
import math
import threading
from uav_delivery import lcmt_quadrotor_setpoint

PORT = 8082
lc = lcm.LCM()

# Shared state
state_lock = threading.Lock()
current_vx = 0.0
current_vy = 0.0
current_vz = 0.0
current_yaw_rate = 0.0
current_yaw = 0.0
last_cmd_time = time.time()

def control_loop():
    global current_yaw, current_vx, current_vy, current_vz, current_yaw_rate, last_cmd_time
    
    dt = 0.02 # 50Hz
    
    while True:
        with state_lock:
            # Timeout safety: if no command received for 0.5s, hover
            if time.time() - last_cmd_time > 0.5:
                current_vx = 0.0
                current_vy = 0.0
                current_vz = 0.0
                current_yaw_rate = 0.0
                
            # Integrate yaw
            current_yaw += current_yaw_rate * dt
            current_yaw = (current_yaw + math.pi) % (2 * math.pi) - math.pi
            
            # Rotate body velocities to world velocities
            vx_W = current_vx * math.cos(current_yaw) - current_vy * math.sin(current_yaw)
            vy_W = current_vx * math.sin(current_yaw) + current_vy * math.cos(current_yaw)
            vz_W = current_vz
            
            msg = lcmt_quadrotor_setpoint()
            msg.utime = int(time.time() * 1e6)
            msg.mode = 1 # Velocity mode
            msg.position = [0.0, 0.0, 0.0]
            msg.velocity = [vx_W, vy_W, vz_W]
            msg.acceleration = [0.0, 0.0, 0.0]
            msg.yaw = current_yaw
            msg.yaw_rate = current_yaw_rate
            
        lc.publish("UAV_QUADROTOR_SETPOINT", msg.encode())
        time.sleep(dt)

# Start background control loop
threading.Thread(target=control_loop, daemon=True).start()

class TeleopRequestHandler(http.server.SimpleHTTPRequestHandler):
    def do_POST(self):
        global current_vx, current_vy, current_vz, current_yaw_rate, last_cmd_time
        if self.path == '/command':
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)
            try:
                data = json.loads(post_data.decode('utf-8'))
                with state_lock:
                    # Left joystick (Throttle = vz, Yaw = yaw_rate)
                    current_vz = -data.get('throttle', 0.0) * 3.0  # Max 3m/s up/down
                    current_yaw_rate = -data.get('yaw', 0.0) * 1.5 # Max 1.5 rad/s
                    
                    # Right joystick (Pitch = vx, Roll = vy)
                    current_vx = data.get('pitch', 0.0) * 4.0      # Max 4m/s forward
                    current_vy = -data.get('roll', 0.0) * 4.0      # Max 4m/s side
                    last_cmd_time = time.time()
            except Exception as e:
                print(f"Error parsing command: {e}")
                
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(b'{"status":"ok"}')
        else:
            self.send_response(404)
            self.end_headers()

with socketserver.TCPServer(("", PORT), TeleopRequestHandler) as httpd:
    print(f"Serving Web Teleop UI on port {PORT}")
    httpd.serve_forever()
