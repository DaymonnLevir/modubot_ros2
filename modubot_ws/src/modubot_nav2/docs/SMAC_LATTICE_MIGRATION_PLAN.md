# Smac Lattice migration plan

## Objective

Replace the circular approximation used by Smac Planner 2D with an SE2 state
lattice that collision-checks the complete ModuBot footprint at each candidate
pose. Keep reverse expansion disabled, preserve in-place rotation, and retain
MPPI as the local controller.

The corrected Smac 2D baseline was preserved in
`config/nav2_params_smac2d.yaml`. The default `config/nav2_params.yaml` now
contains the lattice profile, permitting an A/B comparison and one-argument
rollback while it is validated.

## Confirmed platform constraints

- ROS 2 Humble with Nav2 1.1.20.
- Global costmap resolution: 0.05 m.
- Padded footprint represented by the costmaps:
  `x=[-0.46, 0.18] m`, `y=[-0.24, 0.24] m`.
- Differential drive with in-place rotation.
- Reverse planning and commands remain disabled by default. Controlled tests
  may enable them consistently with the `allow_reverse` launch argument.
- The installed Nav2 package includes the differential primitive set at:
  `/opt/ros/humble/share/nav2_smac_planner/sample_primitives/5cm_resolution/0.5m_turning_radius/diff/output.json`.
  It has 16 headings and 112 trajectories.

## Phase 1: reproducible lattice profile (implemented)

- [x] Vendored the differential 5 cm / 0.5 m primitive JSON in
  `config/lattice/`, with source and checksum recorded beside it.
- [x] Preserved the corrected Smac 2D parameters as
  `config/nav2_params_smac2d.yaml`.
- [x] Made Smac Lattice the default in `config/nav2_params.yaml`.
- [x] Extended the real and simulated launch files to resolve the vendored
  lattice path from the installed package share directory.
- [x] Kept rollback available through the existing `nav2_params_file` launch
  argument, without duplicating the launch implementation.

Physical robot rollback example:

```bash
ros2 launch modubot_nav2 nav2.launch.py \
  nav2_params_file:=/workspace/modubot_ws/install/modubot_nav2/share/modubot_nav2/config/nav2_params_smac2d.yaml
```

Initial lattice parameters:

```yaml
GridBased:
  plugin: "nav2_smac_planner/SmacPlannerLattice"
  tolerance: 0.20
  allow_unknown: false
  max_iterations: 1000000
  max_on_approach_iterations: 1000
  max_planning_time: 2.0
  lattice_filepath: "__MODUBOT_LATTICE_FILE__"
  cache_obstacle_heuristic: false
  reverse_penalty: 5.0
  change_penalty: 0.10
  non_straight_penalty: 1.10
  cost_penalty: 4.0
  retrospective_penalty: 0.025
  rotation_penalty: 5.0
  analytic_expansion_ratio: 3.5
  analytic_expansion_max_length: 2.5
  lookup_table_size: 10.0
  allow_reverse_expansion: false
  smooth_path: false
```

These are starting values, not final tuning. `smooth_path: false` keeps the
first comparison attributable to the state lattice and its full-footprint
collision checks. `rotation_penalty: 5.0` keeps rotation available without
making it a routine shortcut. Obstacle-heuristic caching starts disabled
because the global costmap contains live lidar obstacles.

## Phase 2: static validation before motion

Use identical start and goal poses for both planner profiles.

1. Confirm that the lattice profile loads without fallback or parameter errors.
2. Display `/plan`, `/unsmoothed_plan`, `/global_costmap/costmap`, and
   `/global_costmap/published_footprint` in RViz.
3. Test at least these map situations:
   - straight corridor;
   - narrow door;
   - 90-degree corner;
   - U-shaped region;
   - goal requiring an in-place heading change;
   - static lidar obstacle placed on the original route.
4. Reject any plan whose transformed footprint intersects a lethal or unknown
   cell at any pose.
5. With `allow_reverse:=false`, confirm that no path segment requests reverse
   motion. With it enabled, confirm that reverse speed never exceeds 0.15 m/s.

## Phase 3: simulation comparison

Run ten repetitions of each route with Smac 2D and ten with Smac Lattice. Keep
the map, goals, MPPI parameters, speed, and costmaps unchanged.

Record:

- planning time and planning failures;
- path length;
- minimum planned footprint clearance;
- navigation success and completion time;
- number of replans and recovery rotations;
- minimum observed obstacle clearance;
- whether any narrow door becomes unusable.

The primary acceptance criterion is zero footprint collisions in planned paths.
Secondary criteria are higher minimum clearance and no material loss of route
success or planning frequency.

## Phase 4: ground validation

Repeat five low-speed trials for each of the six situations above. Begin with
the collision monitor enabled and a maximum linear speed of 0.15 m/s. Increase
to 0.25 m/s only after the planned footprint, lidar obstacles, and executed
trajectory remain mutually aligned.

Measure the same metrics as simulation and additionally record AMCL pose,
odometry, battery voltage, commanded velocity, and video. Lattice tuning must
not be used to compensate for an odometry or localization error.

## Promotion and rollback

Keep Smac Lattice as the validated default only when it:

- produces no footprint-invalid paths;
- crosses every required door;
- never requests reverse motion in the default `allow_reverse:=false` profile;
- keeps the 95th-percentile planning time within the 0.5 s replanning period
  on the Jetson;
- improves or preserves minimum clearance in the repeated trials.

Until then, selecting `nav2_params_smac2d.yaml` through `nav2_params_file`
remains the immediate rollback mechanism.
