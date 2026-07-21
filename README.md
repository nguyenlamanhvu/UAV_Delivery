# UAV Delivery

Drake/Bazel C++ quadrotor simulation with LCM-based process separation. The
current setup follows the same broad pattern used in dairlib controller
examples: one process publishes simulated robot state, another process subscribes
to state and publishes commands, and a repo-local LCM spy knows the custom UAV
message types.

## Layout

- `src/quadrotor_sim.cc`: MultibodyPlant simulation process with Drake
  `Propeller` systems.
- `src/quadrotor_se3_controller.cc`: state-driven SE(3) pose controller process.
- `src/quadrotor_waypoint_trajectory.cc`: state-driven waypoint reference
  generator process.
- `src/quadrotor_visualizer.cc`: Meshcat visualizer process for the URDF model.
- `systems/se3_controller.*`: geometric SE(3) controller LeafSystem with LCM
  message input/output ports.
- `systems/waypoint_trajectory_source.*`: timed waypoint interpolation into
  SE(3) reference messages, driven by incoming UAV state.
- `systems/lcm_driven_loop.h`: dairlib-style LCM-driven execution loop.
- `systems/diagram_utils.*`: optional Graphviz SVG export for process diagrams.
- `systems/lcm_systems.*`: LCM message receiver/sender systems.
- `params/quadrotor_params.*`: YAML-serializable parameters.
- `UAV_models/`: Skydio quadrotor URDF and mesh assets.
- `config/quadrotor_sim.yaml`: default model, plant, controller, channel, and runtime
  configuration.
- `lcmtypes/*.lcm`: custom UAV LCM messages and repo-local `uav-lcm-spy`.

## Build

Regular build:

```bash
bazel --batch build --jobs=12 //:quadrotor_sim //:quadrotor_se3_controller //:quadrotor_waypoint_trajectory //:quadrotor_visualizer //lcmtypes:uav-lcm-spy
```

Release build:

```bash
bazel --batch build --config=release --jobs=12 //:quadrotor_sim //:quadrotor_se3_controller //:quadrotor_waypoint_trajectory //:quadrotor_visualizer //lcmtypes:uav-lcm-spy
```

The repo uses Drake v1.51.1 through Bzlmod in `MODULE.bazel`.

## License

Copyright 2026 Nguyen Lam Anh Vu.

Licensed under the Apache License, Version 2.0. See `LICENSE`.

## Run

Terminal 1, start the plant simulation:

```bash
bazel run //:quadrotor_sim
```

Terminal 2, start the SE(3) controller:

```bash
bazel run //:quadrotor_se3_controller
```

Terminal 3, publish waypoint references. This process waits for
`UAV_QUADROTOR_STATE`, advances on every state tick, and uses Drake periodic
events to update/publish `UAV_QUADROTOR_REFERENCE` at
`trajectory.publish_rate`:

```bash
bazel run //:quadrotor_waypoint_trajectory
```

Inspect LCM:

```bash
bazel run //lcmtypes:uav-lcm-spy
```

Optional terminal 4, visualize the URDF in Meshcat:

```bash
bazel run //:quadrotor_visualizer
```

Each binary writes a process diagram SVG on startup using the binary name, for
example `quadrotor_sim.svg`. Override the output path when needed:

```bash
bazel run //:quadrotor_sim -- --diagram_svg=/tmp/quadrotor_sim.svg
bazel run //:quadrotor_se3_controller -- --diagram_svg=/tmp/quadrotor_controller.svg
bazel run //:quadrotor_visualizer -- --diagram_svg=/tmp/quadrotor_visualizer.svg
```

This uses Graphviz `dot`. If `dot` is not installed, the binary still writes a
`.dot` file next to the SVG path. Passing a directory writes
`<binary_name>.svg` inside that directory.

Use the repo-local spy above so Java has the generated
`uav_delivery.lcmt_*` classes on its classpath. A system `lcm-spy` may see the
channels but mark them undecodable.

## LCM Channels

Defined in `config/quadrotor_sim.yaml`:

- `UAV_QUADROTOR_STATE`: `uav_delivery.lcmt_quadrotor_state`
- `UAV_QUADROTOR_COMMAND`: `uav_delivery.lcmt_quadrotor_command`
- `UAV_QUADROTOR_REFERENCE`: `uav_delivery.lcmt_quadrotor_reference`
- `UAV_SIM_TIME`: `uav_delivery.lcmt_sim_time`

The current process graph is:

```text
quadrotor_sim -> UAV_QUADROTOR_STATE
quadrotor_waypoint_trajectory listens to UAV_QUADROTOR_STATE -> UAV_QUADROTOR_REFERENCE
quadrotor_se3_controller -> UAV_QUADROTOR_COMMAND
quadrotor_sim -> UAV_SIM_TIME
quadrotor_visualizer subscribes UAV_QUADROTOR_STATE and renders the URDF in Meshcat
```

The SE(3) controller is driven by `UAV_QUADROTOR_STATE`: it waits for a new
state message, updates the latest waypoint reference if one has arrived,
advances to the message timestamp, and then force-publishes one
`UAV_QUADROTOR_COMMAND`.

## Configuration

Most tuning lives in `config/quadrotor_sim.yaml`:

- Model path: Skydio URDF used by the visualizer and shared process config.
- Plant parameters: mass, gravity, arm length, thrust/yaw coefficients, inertia.
- Initial state: position, initial RPY used to seed `R`, linear velocity, body angular velocity.
- SE(3) controller gains: desired position/velocity/yaw and geometric PD gains.
- Waypoint trajectory: timed position/yaw waypoints and reference publish rate.
- Runtime settings: publish rate, realtime rate, sim time, console logging.

`lcm_url` is intentionally a command-line flag, not YAML:

```bash
bazel run //:quadrotor_sim -- --lcm_url="udpm://239.255.76.67:7667?ttl=0"
```

The waypoint process is state-driven like the controller: it waits for
`UAV_QUADROTOR_STATE`, advances its diagram time to the state timestamp, then
updates its internal reference with a periodic unrestricted update event at
`trajectory.publish_rate`. A periodic LCM publisher sends
`UAV_QUADROTOR_REFERENCE` at the same rate, so the sim/controller can run at
500-1000 Hz while waypoints update at 100 Hz.

## Debug Notes

With no controller running, the simulation receives zero propeller input and the
UAV should fall from its initial `z = 1.0`. With the SE(3) controller running,
the controller subscribes to state and publishes four propeller inputs to hold
the target pose.

To make controller action obvious, edit the YAML, for example:

```yaml
initial_state:
  position: [0.0, 0.0, 0.5]
  rpy: [0.1, -0.1, 0.0]
```

or:

```yaml
se3_controller:
  desired_position: [0.0, 0.0, 1.5]
```
