# UAV Delivery

Drake/Bazel C++ vehicle simulation workspace centered on a quadrotor world that
now combines:
- the newer master-branch Drake `MultibodyPlant` + `Propeller` quadrotor sim
- a teleoperated moving-target ground vehicle
- a unified visualizer that can optionally enable onboard camera rendering and
  RArUco detection

## Layout

### Core simulation path
- `src/quadrotor_sim.cc`: main environment simulation. Uses Drake
  `MultibodyPlant` + `Propeller` for the quadrotor and wires the moving-target
  car into the same process.
- `src/quadrotor_se3_controller.cc`: state-driven SE(3) controller process.
- `src/quadrotor_visualizer.cc`: shared scene visualizer for the drone + moving
  target. Supports optional camera rendering via `--camera_render`.
- `systems/se3_controller.*`: geometric SE(3) controller LeafSystem.
- `systems/lcm_systems.*`: quadrotor LCM receiver/sender systems.
- `systems/diagram_utils.*`: Graphviz SVG export helpers and actor Meshcat path
  helper.
- `systems/sim_utils.*`: SIGINT/termination helpers.

### Moving target path
- `src/moving_target_teleop.cc`: terminal teleop publisher
  (`MOVING_TARGET_TELEOP_CMD`).
- `systems/moving_target_plant.*`: custom 9-state planar car plant.
- `systems/moving_target_controller.*`: teleop + state to left/right torque
  controller.
- `systems/moving_target_lcm_systems.*`: moving-target LCM receiver/sender
  systems.
- `params/moving_target_params.*`: YAML-serializable moving-target parameters.
- `config/moving_target.yaml`: moving-target runtime config.
- `UAV_models/moving_target/`: moving-target URDF assets.

### Camera / detection path
- `params/quadrotor_camera_visualizer_params.h`: camera and detection params.
- `config/quadrotor_target_camera_visualizer.yaml`: camera intrinsics,
  renderer, output paths, and RArUco thresholds.
- `systems/raruco_detector.*`: onboard camera verification and detection
  publisher.
- `systems/uav_image_system.*`: image writer + RTSP streaming bridge for the
  onboard camera feed.
- `UAV_models/moving_target/raruco_depth2_id0.png`: roof marker texture.

### Helper scripts
- `scripts/web_teleop/teleop_server.py`: publishes `UAV_QUADROTOR_SETPOINT`
  from a simple HTTP teleop endpoint on port `8082`.
- `scripts/simulate_joystick.py`: tiny HTTP client that repeatedly posts test
  commands to the web teleop server.
- `scripts/debug_lcm.py`: subscribes to `UAV_QUADROTOR_STATE` and
  `UAV_QUADROTOR_SETPOINT` for quick inspection.

### Shared assets and messages
- `UAV_models/`: quadrotor, moving-target, and environment assets.
- `lcmtypes/*.lcm`: quadrotor, moving-target, and camera detection message
  types.
- `config/quadrotor_sim.yaml`: quadrotor model, plant, controller, channels,
  and runtime config.

## Documentation
- [Tuning, Pre-Run & System Identification](docs/Tuning_and_PreRun.md)
- [Full Control Architecture: SE(3), NMPC & Wind Model](docs/Architecture_SE3_NMPC_Wind.md)

## Build

Regular build:

```bash
bazel --batch build --jobs=12 \
  //:quadrotor_sim \
  //:quadrotor_se3_controller \
  //:quadrotor_nmpc_controller \
  //:quadrotor_visualizer \
  //:moving_target_teleop \
  //lcmtypes:uav-lcm-spy
```

The repo uses Drake v1.51.1 through Bzlmod in `MODULE.bazel`.

If you have another Drake install in `LD_LIBRARY_PATH`, prefer running with:

```bash
env -u LD_LIBRARY_PATH bazel run //:quadrotor_sim
```

## Run

### Terminal 1: start the simulation

```bash
env -u LD_LIBRARY_PATH bazel run //:quadrotor_sim -- \
  --config=config/quadrotor_sim.yaml \
  --moving_target_config=config/moving_target.yaml
```

Useful runtime flags:

```bash
--no_console_log
--run_forever
--diagram_svg=/tmp/quadrotor_sim.svg
```

### Terminal 2: start the SE(3) Geometric controller

```bash
env -u LD_LIBRARY_PATH bazel run //:quadrotor_se3_controller -- \
  --config=config/quadrotor_sim.yaml
```

The controller subscribes to `UAV_QUADROTOR_STATE` and `UAV_QUADROTOR_SETPOINT`.

### Terminal 3: start the NMPC Planner

```bash
env -u LD_LIBRARY_PATH bazel run //:quadrotor_nmpc_controller -- \
  --config=config/quadrotor_sim.yaml
```

The NMPC planner generates optimal local trajectories and pushes commands to the SE(3) controller.

### Terminal 4: optional LCM inspection

```bash
bazel run //lcmtypes:uav-lcm-spy
```

### Terminal 5: shared visualizer without camera rendering

```bash
env -u LD_LIBRARY_PATH bazel run //:quadrotor_visualizer -- \
  --config=config/quadrotor_sim.yaml \
  --moving_target_config=config/moving_target.yaml
```

This starts a Meshcat server and prints the URL on startup. Without
`--camera_render`, the default Meshcat port is `7000`.
By default it also loads the topoexport background from
`models/topoexport_3D_modeling.sdf`.
Disable that with `--nobackground`.
You can switch to another background SDF with `--background_model=...`, for example
`--background_model=models/campus.sdf`.

### Terminal 4 alternative: shared visualizer with the campus background

```bash
env -u LD_LIBRARY_PATH bazel run //:quadrotor_visualizer -- \
  --config=config/quadrotor_sim.yaml \
  --moving_target_config=config/moving_target.yaml \
  --background_model=models/campus.sdf
```

The new background SDF currently uses `models/topoexport_3D_modeling.obj` and
recenters the topoexport model around the world origin so it is visible near the
drone scene.

### Terminal 5 alternative: shared visualizer with onboard camera rendering

```bash
env -u LD_LIBRARY_PATH bazel run //:quadrotor_visualizer -- \
  --config=config/quadrotor_sim.yaml \
  --moving_target_config=config/moving_target.yaml \
  --camera_render \
  --camera_config=config/quadrotor_target_camera_visualizer.yaml
```

Outputs in camera mode:
- raw frames: `/tmp/uav_delivery/drone_front_camera/`
- RArUco overlays: `/tmp/uav_delivery/drone_front_camera_raruco/`
- detection LCM: `UAV_RARUCO_DETECTION`
- RTSP stream: `rtsp://127.0.0.1:8554/Drake_camera_1`

With `--camera_render`, Meshcat uses the port from
`config/quadrotor_target_camera_visualizer.yaml` which is currently `7002`.

### Terminal 6: moving target teleop

```bash
env -u LD_LIBRARY_PATH bazel run //:moving_target_teleop -- \
  --config=config/moving_target.yaml
```

Teleop keys:
- `W/S` or `Up/Down`: throttle
- `A/D` or `Left/Right`: turn
- `Space`: stop
- `Q`: quit

`moving_target_teleop` can start without a TTY, but keyboard control only works
from an interactive terminal.

### Optional Terminal 7: quadrotor web teleop helper

```bash
cd scripts/web_teleop
python3 teleop_server.py
```

This serves `index.html` and publishes `UAV_QUADROTOR_SETPOINT` from HTTP
commands on port `8082`.

### Optional Terminal 8: debug quadrotor state and setpoint traffic

```bash
PYTHONPATH=scripts/web_teleop python3 scripts/debug_lcm.py
```

## Diagram SVG output

Binaries can write a process diagram SVG on startup. Examples:

```bash
bazel run //:quadrotor_sim -- --diagram_svg=/tmp/quadrotor_sim.svg
bazel run //:quadrotor_se3_controller -- --diagram_svg=/tmp/quadrotor_controller.svg
bazel run //:quadrotor_visualizer -- --diagram_svg=/tmp/quadrotor_visualizer.svg
```

Passing a directory writes `<binary_name>.svg` inside that directory. If
Graphviz `dot` is unavailable, the code still writes a `.dot` file.

## LCM Channels

### Quadrotor
- `UAV_QUADROTOR_STATE`: `uav_delivery.lcmt_quadrotor_state`
- `UAV_QUADROTOR_COMMAND`: `uav_delivery.lcmt_quadrotor_command`
- `UAV_QUADROTOR_SETPOINT`: `uav_delivery.lcmt_quadrotor_setpoint`
- `UAV_SIM_TIME`: `uav_delivery.lcmt_sim_time`

### Moving target
- `MOVING_TARGET_TELEOP_CMD`: `uav_delivery.lcmt_moving_target_teleop_command`
- `MOVING_TARGET_ACTUATION_CMD`: `uav_delivery.lcmt_moving_target_actuation_command`
- `MOVING_TARGET_STATE`: `uav_delivery.lcmt_moving_target_state`

### Camera / detection
- `UAV_RARUCO_DETECTION`: `uav_delivery.lcmt_raruco_detection`

## Debug Notes

- The quadrotor simulation uses Drake `MultibodyPlant` and `Propeller` for rotor
  force application.
- The moving target is part of the same simulated world and publishes its own
  LCM state.
- With no teleop running, the moving target should remain near its initial
  state.
- With no controller running, the quadrotor receives zero propeller input and
  should fall from its initial state.
- With no setpoint publisher running, the controller still starts, but it will
  not receive commanded motion updates on `UAV_QUADROTOR_SETPOINT`.
- `quadrotor_visualizer` is the only visualizer entrypoint. Toggle onboard
  rendering with `--camera_render`.

## License

Copyright 2026 Nguyen Lam Anh Vu.

Licensed under the Apache License, Version 2.0. See `LICENSE`.
