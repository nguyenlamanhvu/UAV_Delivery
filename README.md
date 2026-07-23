# UAV Delivery

Drake/Bazel C++ vehicle simulation workspace with a quadrotor environment that
contains a teleoperated moving-target car and LCM-based visualization.

## Layout

### Quadrotor environment path
- `src/quadrotor_sim.cc`: quadrotor environment simulation process. It owns the
  quadrotor plant, moving-target car plant, and car teleop controller wiring.
- `src/quadrotor_se3_controller.cc`: state-driven SE(3) controller process.
- `src/quadrotor_visualizer.cc`: Meshcat visualizer process for the drone + car
  scene.
- `systems/quadrotor_plant.*`: 18-state SE(3) quadrotor dynamics.
- `systems/se3_controller.*`: geometric SE(3) controller LeafSystem with LCM
  message input/output ports.

### moving_target path
- `src/moving_target_teleop.cc`: terminal teleop publisher (`MOVING_TARGET_TELEOP_CMD`).
- `systems/moving_target_plant.*`: custom 9-state planar ground vehicle plant.
- `systems/moving_target_controller.*`: teleop + state to left/right torque
  controller LeafSystem wired inside `quadrotor_sim`.
- `systems/moving_target_lcm_systems.*`: LCM receiver/sender systems for teleop,
  actuation, and state.
- `params/moving_target_params.*`: YAML-serializable moving-target parameters.
- `config/moving_target.yaml`: runtime config for URDF path, plant gains,
  controller gains, channels, actor namespace, and teleop tuning.
- `UAV_models/moving_target/`: 4-wheel URDF asset package.

### Shared-world camera path
- `src/quadrotor_target_camera_visualizer.cc`: shared drone + moving-target
  scene visualizer with an onboard RGBD camera, raw frame dumping, and RArUco
  overlay / detection publishing.
- `systems/raruco_detector.*`: pose-seeded image verification layer that projects
  the moving-target roof marker into the drone camera and publishes
  `UAV_RARUCO_DETECTION`.
- `config/quadrotor_target_camera_visualizer.yaml`: camera intrinsics, output
  paths, and RArUco detector thresholds / marker mount parameters.
- `UAV_models/moving_target/raruco_depth2_id0.png`: generated recursive marker
  texture mounted on the moving target `base_link` roof plate.

### Shared utilities
- `systems/lcm_driven_loop.h`: dairlib-style LCM-driven execution loop.
- `systems/diagram_utils.*`: optional Graphviz SVG export and actor Meshcat path helper.
- `systems/sim_utils.*`: SIGINT/termination helpers.
- `lcmtypes/*.lcm`: custom UAV + moving-target LCM messages and repo-local
  `uav-lcm-spy`.

## Build

Regular build:

```bash
bazel --batch build --jobs=12 \
  //:quadrotor_sim //:quadrotor_se3_controller //:quadrotor_visualizer \
  //:moving_target_teleop \
  //lcmtypes:uav-lcm-spy
```

On Hermes, if Bazel hits service task limits, use the wrapper scope pattern:

```bash
systemd-run --user --scope -p TasksMax=infinity -p MemoryMax=infinity \
  bash -lc 'cd /data/repos/UAV_Delivery && bazel --batch build --jobs=4 \
    //:quadrotor_sim //:quadrotor_visualizer //:moving_target_teleop'
```

The repo uses Drake v1.51.1 through Bzlmod in `MODULE.bazel`.
If you have another Drake install in `LD_LIBRARY_PATH`, run binaries with
`env -u LD_LIBRARY_PATH` so Bazel uses the Drake library from this workspace.

## Run

### Quadrotor
```bash
env -u LD_LIBRARY_PATH bazel run --jobs=3 //:quadrotor_sim -- \
  --config=config/quadrotor_sim.yaml \
     --moving_target_config=config/moving_target.yaml
env -u LD_LIBRARY_PATH bazel run --jobs=3 //:quadrotor_se3_controller -- \
  --config=config/quadrotor_sim.yaml
env -u LD_LIBRARY_PATH bazel run --jobs=3 //:quadrotor_visualizer -- \
  --config=config/quadrotor_sim.yaml \
     --moving_target_config=config/moving_target.yaml
```

`quadrotor_visualizer` is the combined drone environment: it loads the
quadrotor URDF and the moving-target car URDF into one Meshcat scene, then
subscribes to `UAV_QUADROTOR_STATE` and `MOVING_TARGET_STATE`.

### moving_target teleop
Run alongside `quadrotor_sim` to drive the car inside the drone environment:

```bash
env -u LD_LIBRARY_PATH bazel run --jobs=3 //:moving_target_teleop -- \
  --config=config/moving_target.yaml
```

Teleop keys:
- `W/S` or `Up/Down`: throttle
- `A/D` or `Left/Right`: turn
- `Space`: stop
- `Q`: quit

### Shared drone camera on moving target
Run alongside the quadrotor environment state publishers:

```bash
bazel run //:quadrotor_target_camera_visualizer \
  -- --quadrotor_config=config/quadrotor_sim.yaml \
     --moving_target_config=config/moving_target.yaml \
     --camera_config=config/quadrotor_target_camera_visualizer.yaml
```

Outputs:
- raw frames: `/tmp/uav_delivery/drone_front_camera/`
- RArUco overlays: `/tmp/uav_delivery/drone_front_camera_raruco/`
- detection LCM: `UAV_RARUCO_DETECTION`

## LCM Channels

### Quadrotor
- `UAV_QUADROTOR_STATE`: `uav_delivery.lcmt_quadrotor_state`
- `UAV_QUADROTOR_COMMAND`: `uav_delivery.lcmt_quadrotor_command`
- `UAV_SIM_TIME`: `uav_delivery.lcmt_sim_time`

### moving_target
- `MOVING_TARGET_TELEOP_CMD`: `uav_delivery.lcmt_moving_target_teleop_command`
- `MOVING_TARGET_ACTUATION_CMD`: `uav_delivery.lcmt_moving_target_actuation_command`
- `MOVING_TARGET_STATE`: `uav_delivery.lcmt_moving_target_state`

### Shared camera
- `UAV_RARUCO_DETECTION`: `uav_delivery.lcmt_raruco_detection`

## Shared-world direction

The car is part of the quadrotor environment. Physics and car controller wiring
live in `quadrotor_sim`; visualization and camera rendering live in the drone
environment visualizers.

## Debug Notes

- With no teleop running, the moving target receives zero drive torque and should
  remain at its initial state inside `quadrotor_sim`.
- With teleop active, forward throttle should produce positive `x` motion and
  wheel spin in `quadrotor_visualizer`.

## License

Copyright 2026 Nguyen Lam Anh Vu.

Licensed under the Apache License, Version 2.0. See `LICENSE`.
