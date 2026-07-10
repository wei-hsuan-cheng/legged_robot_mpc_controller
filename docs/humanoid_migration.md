# Humanoid MPC Migration Notes

This package starts from the humanoid whole-body MPC implementation in
`wb_humanoid_mpc`.

## Source Code Copied

- `humanoid_common_mpc`: shared model settings, contact models, gait schedule,
  cost and constraint utilities, Pinocchio model helpers, and initializers.
- `humanoid_wb_mpc`: whole-body acceleration MPC dynamics, foot constraints,
  end-effector dynamics costs, and MRT helpers.
- `humanoid_centroidal_mpc`: centroidal MPC dynamics and constraints, kept for
  the later centroidal example migration.

Headers keep their original include prefixes for now:

- `humanoid_common_mpc/...`
- `humanoid_wb_mpc/...`
- `humanoid_centroidal_mpc/...`

This keeps the first migration mechanical and avoids renaming the full core at
the same time as changing the ROS 2 integration.

## Deliberate Changes From `wb_humanoid_mpc`

- No `mujoco_vendor` or `mujoco_sim_interface` path.
- Launch and simulation are through `mujoco_ros2_control`.
- Runtime configuration should move from `task.info` to generated parameter
  YAML, matching `dynamics_mpc_controller`.

## Remaining Integration Work

1. Replace `.info` loading in `ModelSettings`, `WBMpcInterface`, cost factories,
   contact settings, and command settings with generated parameter-library
   structs.
2. Expand the MuJoCo observation adapter if additional floating-base state
   sources are needed beyond `/mujoco/ground_truth/odom`.
3. Decide how to command excluded/fixed joints that are not part of the MPC
   state/input vectors.
4. Add performance visualization after the OCS2 policy and Pinocchio interfaces
   are wired.
