# Schedule

## Target

- Build a **multi-robot** planning and control algorithm framework.
- Realize **centralized** or **distributed** simulation control of multiple robots.

## Architecture

### Centralized

TBD

### Distributed

- Each robot has an independent perception, planning, and control pipeline.

- The inputs are the target position and the odometer.

- The outputs are the control inputs.

- Odometer
  - [x] Use gazebo's ground-truth. The `/odom` topic published by `libgazebo_ros_diff_drive`. Transform from `map` to `robotx/odom`. By `tf_map2odom.py`
  - [ ] Use `amcl` in a locatable environment, which will introduce static obstacles. [P1]
    - Remember to disable the `tf_map2odom`, because `amcl` will publish one.

- Global planner
  - [x] Output a one-point path as the result.

- Local planner [P0]
  - [ ] Subscribe to others' odom.
  - [ ] Calculate new velocities and publish them.

- Bugs
  - [x] Wrong scan topic name in `move_base`.
