# Debug GUI Documentation

The UAV Delivery project features a powerful, modular visualization dashboard built on ImGui, ImPlot, and LCM. It acts as the central hub for debugging, monitoring telemetry, and visualizing state data in real-time.

## Features

### 1. Docking & Grid Layouts
The entire interface supports native **ImGui Docking**. 
- You can add multiple independent plugin windows.
- Drag the title bar of any window to snap it to the left, right, top, or bottom of the screen.
- Split windows into custom grid layouts perfectly suited for your workflow, similar to PlotJuggler.

### 2. Live Data Plotting
Select the **Data Plot** plugin to inspect LCM traffic.
- It parses raw `lcmt_quadrotor_state` and `lcmt_moving_target_state` dynamically.
- Automatically handles variables and vectors (position, velocity, orientation).
- You can clear history, pause plotting, and follow the latest timestamps.
- **Smart Subplots**: If you plot multiple signals (e.g., `x`, `y`, `z` from `position`), they are automatically grouped into a smart subplot grid.

### 3. 2D Map Navigation
Select the **2D Map** plugin to track the drone's position in a top-down view.
- Overlays an interactive 2D map texture (`maps/map_2d_clean_no_trees_no_shadows.png`).
- Tracks the live `position` of the drone (blue triangle) and the moving target (red triangle).
- Hover over the map to view real-world `(X, Y)` coordinates.

### 4. RTSP Camera Streaming
Select the **RTSP Camera** plugin to view live video feeds.
- Connects to an RTSP server (e.g., `rtsp://127.0.0.1:8554/test`).
- Uses a lightweight GStreamer `uridecodebin` pipeline directly integrated into the OpenGL render loop.
- Decodes and renders raw RGBA frames natively inside the ImGui cell without heavy browser dependencies.

### 5. MeshCat Integration
Select the **MeshCat Control** plugin to access 3D visualization.
- Acts as a control panel for the Drake MeshCat server.
- Contains a quick-launch button to open the full MeshCat 3D view natively in your web browser.

## Running the GUI
To build and launch the simulation GUI natively:
```bash
bazel run //gui:simulation_gui
```
