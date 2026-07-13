# UAV Delivery

Drake/Bazel C++ quadrotor simulation with LCM-based process separation. The
current setup follows the same broad pattern used in dairlib controller
examples: one process publishes simulated robot state, another process subscribes
to state and publishes commands, and a repo-local LCM spy knows the custom UAV
message types.

## Layout

- `src/quadrotor_sim.cc`: quadrotor plant simulation process.
- `src/quadrotor_hover_controller.cc`: state-driven hover controller process.
- `systems/quadrotor_plant.*`: 12-state quadrotor dynamics.
- `systems/hover_controller.*`: controller LeafSystem.
- `systems/lcm_systems.*`: LCM message receiver/sender systems.
- `params/quadrotor_params.*`: YAML-serializable parameters.
- `config/quadrotor_sim.yaml`: default plant, controller, channel, and runtime
  configuration.
- `lcmtypes/*.lcm`: custom UAV LCM messages and repo-local `uav-lcm-spy`.

## Build

Regular build:

```bash
bazel --batch build --jobs=12 //:quadrotor_sim //:quadrotor_hover_controller //lcmtypes:uav-lcm-spy
```

Release build:

```bash
bazel --batch build --config=release --jobs=12 //:quadrotor_sim //:quadrotor_hover_controller //lcmtypes:uav-lcm-spy
```

The repo uses Drake v1.51.1 through Bzlmod in `MODULE.bazel`.

## Run

Terminal 1, start the plant simulation:

```bash
bazel run //:quadrotor_sim
```

Terminal 2, start the hover controller:

```bash
bazel run //:quadrotor_hover_controller
```

Terminal 3, inspect LCM:

```bash
bazel run //lcmtypes:uav-lcm-spy
```

Use the repo-local spy above so Java has the generated
`uav_delivery.lcmt_*` classes on its classpath. A system `lcm-spy` may see the
channels but mark them undecodable.

## LCM Channels

Defined in `config/quadrotor_sim.yaml`:

- `UAV_QUADROTOR_STATE`: `uav_delivery.lcmt_quadrotor_state`
- `UAV_QUADROTOR_COMMAND`: `uav_delivery.lcmt_quadrotor_command`
- `UAV_SIM_TIME`: `uav_delivery.lcmt_sim_time`

The current process graph is:

```text
quadrotor_sim -> UAV_QUADROTOR_STATE
quadrotor_hover_controller -> UAV_QUADROTOR_COMMAND
quadrotor_sim -> UAV_SIM_TIME
```

## Configuration

Most tuning lives in `config/quadrotor_sim.yaml`:

- Plant parameters: mass, gravity, arm length, thrust/yaw coefficients, inertia.
- Initial state: position, RPY, linear velocity, body angular velocity.
- Hover controller gains: desired altitude, attitude gains, altitude gains.
- Runtime settings: publish rate, realtime rate, sim time, console logging.

`lcm_url` is intentionally a command-line flag, not YAML:

```bash
bazel run //:quadrotor_sim -- --lcm_url="udpm://239.255.76.67:7667?ttl=0"
```

## Debug Notes

With no controller running, the simulation receives zero rotor command and the
UAV should fall from its initial `z = 1.0`. With the hover controller running,
the controller subscribes to state and publishes rotor speeds to hold the target
altitude.

To make controller action obvious, edit the YAML, for example:

```yaml
initial_state:
  position: [0.0, 0.0, 0.5]
  rpy: [0.1, -0.1, 0.0]
```

or:

```yaml
hover_controller:
  desired_z: 1.5
```
