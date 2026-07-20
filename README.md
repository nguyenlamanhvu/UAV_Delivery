# UAV Delivery

Drake/Bazel C++ vehicle simulation workspace with LCM-based process separation.
The original quadrotor stack remains intact, and this repo now also includes a
`moving_target` ground vehicle path designed to be future-compatible with a
shared drone + ground-actor world.

## Layout

### Quadrotor path
- `src/quadrotor_sim.cc`: quadrotor plant simulation process.
- `src/quadrotor_se3_controller.cc`: state-driven SE(3) controller process.
- `src/quadrotor_visualizer.cc`: Meshcat visualizer process for the URDF model.
- `systems/quadrotor_plant.*`: 18-state SE(3) quadrotor dynamics.
- `systems/se3_controller.*`: geometric SE(3) controller LeafSystem with LCM
  message input/output ports.

### moving_target path
- `src/moving_target_teleop.cc`: terminal teleop publisher (`MOVING_TARGET_TELEOP_CMD`).
- `src/moving_target_controller.cc`: teleop-to-actuation controller process.
- `src/moving_target_sim.cc`: planar moving-target dynamics simulator.
- `src/moving_target_visualizer.cc`: Meshcat visualizer for the moving-target URDF.
- `systems/moving_target_plant.*`: custom 9-state planar ground vehicle plant.
- `systems/moving_target_controller.*`: throttle/turn to left/right torque mixer.
- `systems/moving_target_lcm_systems.*`: LCM receiver/sender systems for teleop,
  actuation, and state.
- `params/moving_target_params.*`: YAML-serializable moving-target parameters.
- `config/moving_target.yaml`: runtime config for URDF path, plant gains,
  controller gains, channels, actor namespace, and teleop tuning.
- `UAV_models/moving_target/`: 4-wheel URDF asset package.

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
  //:moving_target_teleop //:moving_target_controller //:moving_target_sim //:moving_target_visualizer \
  //lcmtypes:uav-lcm-spy
```

On Hermes, if Bazel hits service task limits, use the wrapper scope pattern:

```bash
systemd-run --user --scope -p TasksMax=infinity -p MemoryMax=infinity \
  bash -lc 'cd /data/repos/UAV_Delivery && bazel --batch build --jobs=4 \
    //:moving_target_teleop //:moving_target_controller //:moving_target_sim //:moving_target_visualizer'
```

The repo uses Drake v1.51.1 through Bzlmod in `MODULE.bazel`.

## Run

### Quadrotor
```bash
bazel run //:quadrotor_sim
bazel run //:quadrotor_se3_controller
bazel run //:quadrotor_visualizer
```

### moving_target
Use four terminals:

```bash
# Terminal 1
bazel run //:moving_target_visualizer -- --config=config/moving_target.yaml

# Terminal 2
bazel run //:moving_target_sim -- --config=config/moving_target.yaml

# Terminal 3
bazel run //:moving_target_controller -- --config=config/moving_target.yaml

# Terminal 4
bazel run //:moving_target_teleop -- --config=config/moving_target.yaml
```

Teleop keys:
- `W/S` or `Up/Down`: throttle
- `A/D` or `Left/Right`: turn
- `Space`: stop
- `Q`: quit

## LCM Channels

### Quadrotor
- `UAV_QUADROTOR_STATE`: `uav_delivery.lcmt_quadrotor_state`
- `UAV_QUADROTOR_COMMAND`: `uav_delivery.lcmt_quadrotor_command`
- `UAV_SIM_TIME`: `uav_delivery.lcmt_sim_time`

### moving_target
- `MOVING_TARGET_TELEOP_CMD`: `uav_delivery.lcmt_moving_target_teleop_command`
- `MOVING_TARGET_ACTUATION_CMD`: `uav_delivery.lcmt_moving_target_actuation_command`
- `MOVING_TARGET_STATE`: `uav_delivery.lcmt_moving_target_state`

## Shared-world direction

The moving-target path is namespaced for future multi-actor visualization:
- Meshcat actor roots should live under `/actors/<name>`
- default ground vehicle root: `/actors/moving_target`
- future drone root can become `/actors/quadrotor`

Today the stacks still run as separate actor-local apps over isolated LCM
channels, but the naming and visualization conventions are now aligned so a
future unified visualizer can subscribe to both state channels and place both
actors into one Meshcat scene without rewriting the message topology.

## Debug Notes

- With no teleop/controller running, `moving_target_sim` receives zero drive
  torque and should remain at its initial state.
- With teleop + controller active, forward throttle should produce positive `x`
  motion and wheel spin logs in the visualizer.
- `moving_target_visualizer` uses a 4-wheel URDF, but motion still comes from
  the custom planar plant; this mismatch is intentional for v1.

## License

Copyright 2026 Nguyen Lam Anh Vu.

Licensed under the Apache License, Version 2.0. See `LICENSE`.
