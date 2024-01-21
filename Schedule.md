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
  - [ ] Use gazebo's ground-truth.
  - [ ] Use `amcl` in a locatable environment, which will introduce static obstacles.

- Global planner
  - [ ] Output a one-point path as the result.

- Local planner
  - [ ] Subscribe to others' odom.
  - [ ] Calculate new velocities and publish them.
