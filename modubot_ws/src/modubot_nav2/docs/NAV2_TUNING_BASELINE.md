# ModuBot Nav2 tuning baseline

This document records the reasoning behind the conservative baseline in
`config/nav2_params.yaml`. The values are intended for the first integrated
ground test at a maximum linear speed of 0.20 m/s. They are not a substitute
for the physical odometry and braking calibrations listed below.

## Baseline decisions

### Map and global planning

- The global costmap tracks unknown space and Smac Planner 2D is not allowed
  to plan through it. Unmapped cells are therefore non-traversable.
- Smac Planner 2D uses cost-aware A* on the native 0.05 m costmap. A travel
  cost multiplier of 2.0 favors the center of the inflation potential without
  making planning unnecessarily expensive.
- The global obstacle layer uses `combination_method: 2`, preserving unknown
  cells from the static map instead of clearing them with lidar raytracing.
- The tracked map used in an experiment must be stored in this repository. The
  current default remains `maps/piso01.yaml`; a map passed through the `map`
  launch argument must also be archived before an article experiment.

### Lidar

- Valid infinite lidar returns remain available for costmap clearing.
- The mechanically blocked rear sector and returns below the trusted minimum
  range are converted to NaN by `simple_filter.py`. NaN represents an
  unobserved ray and prevents the blind sector from being cleared as free.
- The useful field of view remains approximately 330 degrees. The rear blind
  sector must be considered when testing reverse recovery and in-place turns.

### AMCL

- Updates require 0.05 m or 0.05 rad, reducing the previous sensitivity to
  encoder quantization and caster-induced oscillations.
- Ninety evenly spaced beams provide more information than the Nav2 example
  baseline without retaining the previous 120-beam processing load.
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
- Both inflation layers use a 0.55 m radius and 2.5 scaling factor. This keeps a
  useful potential field while restoring passage through doors that became
  impractical with the former 0.75/0.85 m radii and 1.4 scaling factor.
- Lidar observations persist for 0.25 s. This retains short clusters such as
  legs for multiple costmap cycles without leaving half-second ghost obstacles.

### MPPI and velocity smoothing

- Maximum speed is 0.20 m/s and maximum angular speed is 0.8 rad/s.
- `nav2_mppi_controller` is built natively from the vendored Nav2 1.1.20
  source. This workspace overlay avoids the known `SIGILL` failure of the
  Humble ARM64 binary during noise-generator initialization on Jetson.
- MPPI runs at 20 Hz with a 0.05 s model interval. Sixty model steps provide a
  3.0 s prediction horizon, equivalent to 0.60 m at maximum linear speed.
- The initial batch contains 2000 sampled trajectories. This intentionally
  starts at the high-quality end for the Jetson benchmark; reduce it to 1500,
  1000 or 750 only if the controller misses its 20 Hz deadline.
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
- The slowdown zone reaches 0.65 m forward and scales both linear and angular
  commands to 65% after at least three scan points are present.
- Full stop is restricted to the 0.18--0.23 m frontal strip and requires at
  least four scan points. This prevents isolated returns or normal lateral
  caster drift from continuously locking all differential-drive motion.
- Recovery waiting is two seconds and backup distance is 0.20 m. Spin and
  backup limits are explicitly aligned with the robot's velocity and
  angular limits.

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
     measurement. This value may differ from the geometric 0.207 m because of
     tire scrub, load distribution and caster forces.

4. **PI response and braking**
   - Measure rise time, overshoot and settling at 0.05, 0.10, 0.15 and 0.20 m/s.
   - Measure stopping distance from 0.20 m/s on the intended floor.
   - Update PI, acceleration/deceleration limits and the emergency-stop depth
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
