#!/bin/bash
# run_all.sh - The All-In-One Launcher for UAV_Delivery

echo "==========================================================="
echo "   Starting UAV Delivery Simulation Stack (Full System)    "
echo "==========================================================="

echo "Cleaning up stale ports (7000, 8554, 5000)..."
fuser -k -9 7000/tcp 2>/dev/null
fuser -k -9 8554/tcp 2>/dev/null
fuser -k -9 5000/tcp 2>/dev/null
sleep 1

# Ensure bazel is ready
bazel build //:quadrotor_sim //:quadrotor_visualizer //:quadrotor_se3_controller //:moving_target_teleop || exit 1

# 1. Start the 3D Visualizer + Camera Rendering
echo "[1/5] Starting Quadrotor Visualizer (Meshcat on 7000, Camera active)..."
bazel run //:quadrotor_visualizer -- --camera_render=true &
VIS_PID=$!
sleep 2

# 2. Start the Drone Flight Controller
echo "[2/5] Starting Quadrotor SE(3) Controller..."
bazel run //:quadrotor_se3_controller &
CTRL_PID=$!
sleep 1

# 3. Start the Moving Target Controller
echo "[3/5] Starting Moving Target Teleop Controller..."
bazel run //:moving_target_teleop &
MT_PID=$!
sleep 1

# 4. Start the Main Physics Simulation (with Wind Engine attached)
echo "[4/5] Starting Main Physics Simulation (quadrotor_sim)..."
bazel run //:quadrotor_sim -- --no_console_log=true &
SIM_PID=$!
sleep 2

# 5. Start the Ultimate Command Center
echo "[5/5] Starting Ultimate Command Center Web UI..."
echo "-----------------------------------------------------------"
echo "      Go to: http://localhost:5000 in your browser!        "
echo "      (Or http://localhost:7000 for the Meshcat 3D view)   "
echo "-----------------------------------------------------------"
echo "Generating Python LCM Bindings..."
lcm-gen -p lcmtypes/*.lcm
pip3 install flask opencv-python --break-system-packages > /dev/null 2>&1
export PYTHONPATH=$PYTHONPATH:$(pwd)
python3 scripts/wind_gui_server.py

# Cleanup child processes when you hit Ctrl+C on the python script
echo "Shutting down simulation stack..."
kill $VIS_PID $CTRL_PID $MT_PID $SIM_PID
wait $VIS_PID $CTRL_PID $MT_PID $SIM_PID 2>/dev/null
echo "Shutdown complete."
