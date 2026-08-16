# ModuBot differential-drive motion primitives

`modubot_diff_5cm_r0p5.json` is the differential-drive state-lattice primitive
set distributed with Nav2 1.1.20 under the Apache License 2.0. It was copied
from:

`nav2_smac_planner/sample_primitives/5cm_resolution/0.5m_turning_radius/diff/output.json`

The set matches the 0.05 m global costmap resolution and provides 112
trajectories over 16 heading bins, including in-place rotations. Its nominal
minimum turning radius is 0.5 m. Reverse expansion is disabled separately in
the ModuBot Nav2 parameters.

SHA-256:
`105b37351d15e340cc2921fa8878c8ea727a65cd7e89c39c977be0654be3b2a9`

Do not edit this generated JSON manually. If the costmap resolution, motion
model, or primitive geometry changes, generate and validate a new primitive
set and update this provenance record.
