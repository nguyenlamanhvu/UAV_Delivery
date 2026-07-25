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
- `UAV_models/moving_target/raruco_depth2_id0.png`: roof marker texture.

### Shared assets and messages
- `UAV_models/`: quadrotor, moving-target, and environment assets.
- `lcmtypes/*.lcm`: quadrotor, moving-target, and camera detection message
  types.
- `config/quadrotor_sim.yaml`: quadrotor model, plant, controller, channels,
  and runtime config.

## Build

Regular build:

```bash
bazel --batch build --jobs=12 \
  //:quadrotor_sim \
  //:quadrotor_se3_controller \
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

### Terminal 2: start the SE(3) controller

```bash
env -u LD_LIBRARY_PATH bazel run //:quadrotor_se3_controller -- \
  --config=config/quadrotor_sim.yaml
```

### Terminal 3: optional LCM inspection

```bash
bazel run //lcmtypes:uav-lcm-spy
```

### Terminal 4: shared visualizer without camera rendering

```bash
env -u LD_LIBRARY_PATH bazel run //:quadrotor_visualizer -- \
  --config=config/quadrotor_sim.yaml \
  --moving_target_config=config/moving_target.yaml
```

### Terminal 4 alternative: shared visualizer with onboard camera rendering

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

### Terminal 5: moving target teleop

```bash
env -u LD_LIBRARY_PATH bazel run //:moving_target_teleop -- \
  --config=config/moving_target.yaml
```

Teleop keys:
- `W/S` or `Up/Down`: throttle
- `A/D` or `Left/Right`: turn
- `Space`: stop
- `Q`: quit

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
- `quadrotor_visualizer` is the only visualizer entrypoint. Toggle onboard
  rendering with `--camera_render`.

## License

Copyright 2026 Nguyen Lam Anh Vu.

Licensed under the Apache License, Version 2.0. See `LICENSE`.
