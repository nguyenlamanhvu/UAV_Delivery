# moving_target URDF

Simple four-wheel ground vehicle asset for the `moving_target_*` stack.

## Link names
- `base_link`
- `front_left_wheel_link`
- `front_right_wheel_link`
- `rear_left_wheel_link`
- `rear_right_wheel_link`

## Joint names
- `front_left_wheel_joint`
- `front_right_wheel_joint`
- `rear_left_wheel_joint`
- `rear_right_wheel_joint`

All wheel joints are `continuous` and spin about local `+Y`.

## Dimensions
- Chassis: `0.8 x 0.5 x 0.2 m`
- Wheel radius: `0.12 m`
- Wheel thickness: `0.06 m`

For v1 the URDF is primarily visual/structural. Vehicle motion comes from the
custom planar Drake plant in `systems/moving_target_plant.*`.
