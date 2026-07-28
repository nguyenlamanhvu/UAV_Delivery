import os
import sys
import threading
import time
import math
import cv2
from flask import Flask, request, jsonify, send_file, Response
import lcm

try:
    from uav_delivery import lcmt_wind_parameters, lcmt_quadrotor_setpoint, lcmt_camera_command
except ImportError:
    print("Warning: LCM types not found in PYTHONPATH.")

app = Flask(__name__, static_url_path='', static_folder='.')
lc = lcm.LCM()

# --- Flight Control State ---
state_lock = threading.Lock()
current_vx = 0.0
current_vy = 0.0
current_vz = 0.0
current_yaw_rate = 0.0
current_yaw = 0.0
current_camera_pitch = 25.0
target_pos = [0.0, 0.0, 1.0]
last_cmd_time = time.time()

current_wind_nominal = [0.0, 0.0, 0.0]
current_wind_sigma = 0.0
current_wind_altitude = 1.0
current_wind_drag_coeff = 0.0

def control_loop():
    global current_yaw, current_vx, current_vy, current_vz, current_yaw_rate, last_cmd_time, current_camera_pitch, target_pos
    global current_wind_nominal, current_wind_sigma, current_wind_altitude, current_wind_drag_coeff
    dt = 0.02 # 50Hz
    while True:
        with state_lock:
            # Timeout safety: hover if no command received
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
            
            try:
                # Flight Setpoint
                target_pos[0] += vx_W * dt
                target_pos[1] += vy_W * dt
                target_pos[2] += vz_W * dt
                
                msg = lcmt_quadrotor_setpoint()
                msg.utime = int(time.time() * 1e6)
                msg.mode = 1 # Velocity mode
                msg.position = list(target_pos)
                msg.velocity = [vx_W, vy_W, vz_W]
                msg.acceleration = [0.0, 0.0, 0.0]
                msg.yaw = current_yaw
                msg.yaw_rate = current_yaw_rate
                lc.publish("UAV_QUADROTOR_SETPOINT", msg.encode())
                
                # Camera Command
                cam_msg = lcmt_camera_command()
                cam_msg.pitch_rad = current_camera_pitch
                lc.publish("CAMERA_COMMAND", cam_msg.encode())
            except NameError as e:
                print("LCM Error:", e)
                pass # lcmt types missing
                
        time.sleep(dt)

threading.Thread(target=control_loop, daemon=True).start()

# --- Flask Routes ---
@app.route('/')
def index():
    return send_file('wind_gui.html')

@app.route('/command', methods=['POST'])
def command():
    global current_vx, current_vy, current_vz, current_yaw_rate, last_cmd_time, current_camera_pitch
    data = request.json
    if any(abs(float(data.get(k, 0.0))) > 0.01 for k in ['throttle', 'yaw', 'pitch', 'roll']):
        print("DEBUG COMMAND RECEIVED:", data)
    try:
        with state_lock:
            current_vz = -float(data.get('throttle', 0.0)) * 3.0
            current_yaw_rate = -float(data.get('yaw', 0.0)) * 1.5
            current_vx = float(data.get('pitch', 0.0)) * 4.0
            current_vy = -float(data.get('roll', 0.0)) * 4.0
            current_camera_pitch = float(data.get('camera_pitch', 0.0))
            last_cmd_time = time.time()
        return jsonify({"status": "ok"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 400

@app.route('/update_wind', methods=['POST'])
def update_wind():
    global current_wind_nominal, current_wind_sigma, current_wind_altitude, current_wind_drag_coeff
    data = request.json
    try:
        with state_lock:
            current_wind_nominal = [float(data.get('nominal_x', 0.0)), 
                                    float(data.get('nominal_y', 0.0)), 0.0]
            current_wind_sigma = float(data.get('sigma', 0.0))
            current_wind_altitude = float(data.get('altitude', 1.0))
            current_wind_drag_coeff = float(data.get('drag_coeff', 0.0))
            
            msg = lcmt_wind_parameters()
            msg.nominal_wind = current_wind_nominal
            msg.gust_sigma = [current_wind_sigma, current_wind_sigma, current_wind_sigma]
            msg.altitude = current_wind_altitude
            msg.drag_coeff = current_wind_drag_coeff
            lc.publish("WIND_PARAMETERS", msg.encode())
            
        return jsonify({"status": "success"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500


# --- Background Video Thread ---
latest_frame = None

def capture_video():
    global latest_frame
    cap = cv2.VideoCapture("rtsp://127.0.0.1:8554/Drake_camera_1", cv2.CAP_FFMPEG)
    if not cap.isOpened():
        cap = cv2.VideoCapture("udpsrc port=8554 ! application/x-rtp, payload=96 ! rtph264depay ! avdec_h264 ! videoconvert ! appsink", cv2.CAP_GSTREAMER)
    
    # Try to reduce buffer size if possible
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    
    while True:
        success, frame = cap.read()
        if success:
            ret, buffer = cv2.imencode('.jpg', frame)
            if ret:
                latest_frame = buffer.tobytes()
        else:
            time.sleep(0.01)

threading.Thread(target=capture_video, daemon=True).start()

def gen_frames():
    global latest_frame
    while True:
        if latest_frame is not None:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + latest_frame + b'\r\n')
        time.sleep(0.033) # Max 30 FPS to client


@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    print("Starting Ultimate Command Center on http://localhost:5000")
    app.run(host='0.0.0.0', port=5000, debug=True, threaded=True)
