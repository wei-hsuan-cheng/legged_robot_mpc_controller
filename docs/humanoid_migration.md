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
2. Build an observation adapter:
   - floating base from `/mujoco/ground_truth/odom`;
   - actuated joints from controller state interfaces.
3. Build a command adapter:
   - map MPC joint torque output to ros2_control effort command interfaces;
   - keep a guarded safe hold mode before MPC policy is valid.
4. Add performance visualization after the OCS2 policy and Pinocchio interfaces
   are wired.

