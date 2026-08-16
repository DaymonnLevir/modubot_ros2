# Smac Lattice migration plan

## Objective

Replace the circular approximation used by Smac Planner 2D with an SE2 state
lattice that collision-checks the complete ModuBot footprint at each candidate
pose. Keep reverse expansion disabled, preserve in-place rotation, and retain
MPPI as the local controller.

The migration is intentionally separated from the current Smac 2D correction.
This permits an A/B comparison and a one-argument rollback while the lattice
profile is being validated.

## Confirmed platform constraints

- ROS 2 Humble with Nav2 1.1.20.
- Global costmap resolution: 0.05 m.
- Padded footprint represented by the costmaps:
  `x=[-0.46, 0.18] m`, `y=[-0.24, 0.24] m`.
- Differential drive with in-place rotation.
- Reverse planning and reverse commands must remain disabled.
- The installed Nav2 package includes the differential primitive set at:
  `/opt/ros/humble/share/nav2_smac_planner/sample_primitives/5cm_resolution/0.5m_turning_radius/diff/output.json`.
  It has 16 headings and 112 trajectories.

## Phase 1: reproducible lattice profile

1. Copy the differential 5 cm / 0.5 m primitive JSON into
   `modubot_nav2/config/lattice/` so experiments do not depend on the contents
   of a particular binary installation.
2. Add a separate `nav2_params_lattice.yaml`; do not replace the known Smac 2D
   profile during initial validation.
3. Extend the launch file to rewrite a lattice-file placeholder to the package
   share path, in the same way it currently rewrites the behavior-tree paths.
4. Expose a launch argument selecting `smac_2d` or `smac_lattice`, with
   `smac_2d` retained as the rollback default until the acceptance tests pass.

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
5. Confirm that no path segment requests reverse motion.

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

Promote Smac Lattice to the default only when it:

- produces no footprint-invalid paths;
- crosses every required door;
- never requests reverse motion;
- keeps the 95th-percentile planning time within the 0.5 s replanning period
  on the Jetson;
- improves or preserves minimum clearance in the repeated trials.

Until then, selecting the Smac 2D parameter profile remains the immediate
rollback mechanism.
