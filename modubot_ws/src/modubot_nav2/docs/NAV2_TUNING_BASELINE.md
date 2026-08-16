# ModuBot Nav2 tuning baseline

This document records the reasoning behind the conservative baseline in
`config/nav2_params.yaml`. The values are intended for the first integrated
ground test at a maximum linear speed of 0.25 m/s. They are not a substitute
for the physical odometry and braking calibrations listed below.

## Baseline decisions

### Map and global planning

- The global costmap tracks unknown space and Smac Planner 2D is not allowed
  to plan through it. Unmapped cells are therefore non-traversable.
- Smac Planner 2D uses cost-aware A* downsampled from 0.05 m to 0.10 m. This
  reduces the search grid to one quarter of its native cell count. A travel
  cost multiplier of 4.0 retains a preference for corridor centers.
- The global obstacle layer uses `combination_method: 2`, preserving unknown
  cells from the static map instead of clearing them with lidar raytracing.
- The tracked map used in an experiment must be stored in this repository. The
  current default is `maps/PisoInferiorDC.yaml`; a map passed through the `map`
  launch argument must also be archived before an article experiment.

### Lidar

- Valid infinite lidar returns remain available for costmap clearing.
- The mechanically blocked rear sector and returns below the trusted minimum
  range are converted to NaN by `simple_filter.py`. NaN represents an
  unobserved ray and prevents the blind sector from being cleared as free.
- The useful field of view remains approximately 330 degrees. The rear blind
  sector must be considered when testing reverse recovery and in-place turns.

### AMCL

- Updates require 0.08 m or 0.08 rad, reducing sensitivity to
  encoder quantization and caster-induced oscillations.
- Eighty evenly spaced beams preserve broad scan coverage while reducing the
  particle-filter processing load.
- `likelihood_field_prob` is used so the enabled beam-skipping parameters are
  actually applied in environments containing people and other dynamic
  obstacles.
- Odometry noise values are deliberately above the generic 0.2 defaults. They
  represent the uncertain effective wheel geometry, traction and rear casters.
- The configured initial pose `(0, 0, 0)` is valid only if the robot is placed
  at a physically marked start pose corresponding to that map coordinate.

### Footprint and costmaps

- The physical footprint remains `x=[-0.42, 0.14] m`, `y=[-0.20, 0.20] m`.
- Padding is 0.04 m. It is a collision margin, while inflation is a path-cost
  preference; these roles should not be mixed.
- Global inflation uses a 0.70 m radius and 3.0 scaling factor to favor corridor
  centers without making doors unnecessarily costly. Local inflation uses a
  0.45 m radius and 4.0 scaling factor for a shorter avoidance gradient.
- Lidar observations persist for 0.25 s. This retains short clusters such as
  legs for multiple costmap cycles without leaving half-second ghost obstacles.

### MPPI and velocity smoothing

- Maximum speed is 0.25 m/s and maximum angular speed is 0.8 rad/s.
- `nav2_mppi_controller` is built natively from the vendored Nav2 1.1.20
  source. This workspace overlay avoids the known `SIGILL` failure of the
  Humble ARM64 binary during noise-generator initialization on Jetson.
- MPPI runs at 10 Hz with a 0.10 s model interval. Forty-eight model steps
  provide a 4.8 s prediction horizon, equivalent to 1.20 m at maximum speed.
- The batch contains 700 sampled trajectories. The resulting 33,600 trajectory
  states per cycle are 44% fewer than the previous 800 x 75 configuration.
- One optimization iteration and pre-generated noise minimize runtime jitter.
  Trajectory visualization remains disabled during navigation.
- The differential-drive model uses the full rectangular footprint for
  collision scoring. Cost, path and goal critics permit local obstacle
  avoidance while preserving the intent of the Smac global path.
- The velocity smoother removes commands below 0.03 m/s or 0.10 rad/s. These
  values are also supplied to the MPPI deadband critic and must be replaced by
  measured ground deadbands.
- Progress is 0.10 m within 15 s. This allows low-speed alignment and obstacle
  negotiation without accepting a robot that is genuinely stuck.

### Collision monitor and recoveries

- The monitor is an independent final safety layer, not the normal obstacle
  avoidance controller.
- The only monitor polygon is a final emergency-stop strip in front of the
  robot. In the `base_link` convention, `+x` points longitudinally toward the
  narrow front face and `y` spans the robot width. The strip covers
  `x=[0.18, 0.40] m` and `y=[-0.30, 0.30] m`.
- On Humble, `max_points: 1` triggers after at least two scan readings enter
  the strip. A trigger zeros both linear and angular commands; normal obstacle
  avoidance and early speed selection remain the MPPI and costmap task.
- Reverse recovery is disabled because the mechanically blocked rear lidar
  sector cannot support it safely. Recovery alternates 90-degree rotations in
  both directions and waiting. In Humble, a zero spin simulation horizon is
  used to keep this explicitly accepted in-place rotation available.
- The serial bridge also rejects commands with negative linear velocity, so
  teleoperation and external applications cannot bypass the no-reverse policy.
  Commands with zero linear velocity and nonzero angular velocity remain valid.

## Required physical calibrations

Perform these in order. Change one parameter family at a time and keep the
same map, start pose, battery state and test route for comparisons.

1. **Minimum executable velocity**
   - On the ground, command straight speeds from 0.02 to 0.12 m/s in 0.01 m/s
     increments in both directions.
   - Repeat with pure angular commands from 0.15 to 0.80 rad/s.
   - Record the lowest command that starts promptly in every repetition.
   - Update the MPPI `VelocityDeadbandCritic` and smoother deadbands.

2. **Wheel radius / distance scale**
   - Execute at least ten straight 1.5 m odometry trials in both directions.
   - Compare odometry distance with an external measurement or overhead tag.
   - Adjust effective radius or a dedicated distance multiplier, not the known
     Hall tick count, unless the tick count itself is proven wrong.

3. **Effective wheel separation**
   - Execute repeated 360-degree rotations in both directions.
   - Adjust effective separation until odometry rotation matches the external
     measurement. The current effective value is 0.225 m and may differ from
     the geometric wheel-center distance because of
     tire scrub, load distribution and caster forces.

4. **PI response and braking**
   - Measure rise time, overshoot and settling at 0.05, 0.10, 0.15, 0.20 and 0.25 m/s.
   - Measure stopping distance from 0.25 m/s on the intended floor.
   - Update PI, acceleration/deceleration limits and the slowdown-zone depth
     from these measurements.

5. **Localization-only validation**
   - Drive by teleoperation before sending Nav2 goals.
   - Verify that lidar points stay aligned with mapped walls and that AMCL does
     not jump when the casters align or people cross the scan.
   - Tune AMCL odometry noise only after the odometry scale is calibrated.

6. **MPPI and safety integration**
   - Use a fixed route containing a corridor, a door, a static obstacle and a
     walking-person crossing.
   - First evaluate global/local costmaps and MPPI; then inspect monitor events.
   - Run at least ten repetitions per final configuration and record success,
     travel time, minimum clearance, recoveries, stops and localization error.

## Parameters that remain provisional

- Effective wheel radius and wheel separation.
- PI gains and actual braking deceleration.
- Linear and angular deadbands.
- AMCL odometry noise coefficients.
- Collision-zone depth derived from measured latency and stopping distance.
